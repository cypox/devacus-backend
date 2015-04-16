/*

Copyright (c) 2015, Project OSRM, Dennis Luxen, others
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "processing_chain.hpp"

#include "contractor.hpp"

#include "../algorithms/crc32_processor.hpp"
#include "../data_structures/deallocating_vector.hpp"
#include "../data_structures/static_rtree.hpp"
#include "../data_structures/restriction_map.hpp"

#include "../Util/git_sha.hpp"
#include "../Util/graph_loader.hpp"
#include "../Util/integer_range.hpp"
#include "../Util/lua_util.hpp"
#include "../Util/make_unique.hpp"
#include "../Util/osrm_exception.hpp"
#include "../Util/simple_logger.hpp"
#include "../Util/string_util.hpp"
#include "../Util/timing_util.hpp"
#include "../typedefs.h"

#include <boost/filesystem/fstream.hpp>
#include <boost/program_options.hpp>

#include <tbb/task_scheduler_init.h>
#include <tbb/parallel_sort.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

Prepare::Prepare() : requested_num_threads(1) {}

Prepare::~Prepare() {}

int Prepare::Process(int argc, char *argv[])
{
	LogPolicy::GetInstance().Unmute();
	TIMER_START(preparing);
	TIMER_START(expansion);

	if (!ParseArguments(argc, argv))
	{
		return 0;
	}
	if (!boost::filesystem::is_regular_file(input_path))
	{
		SimpleLogger().Write(logWARNING) << "Input file " << input_path.string() << " not found!";
		return 1;
	}

	if (!boost::filesystem::is_regular_file(profile_path))
	{
		SimpleLogger().Write(logWARNING) << "Profile " << profile_path.string() << " not found!";
		return 1;
	}

	if (1 > requested_num_threads)
	{
		SimpleLogger().Write(logWARNING) << "Number of threads must be 1 or larger";
		return 1;
	}

	const unsigned recommended_num_threads = tbb::task_scheduler_init::default_num_threads();

	SimpleLogger().Write() << "Input file: " << input_path.filename().string();
	SimpleLogger().Write() << "Restrictions file: " << restrictions_path.filename().string();
	SimpleLogger().Write() << "Profile: " << profile_path.filename().string();
	SimpleLogger().Write() << "Threads: " << requested_num_threads;
	if (recommended_num_threads != requested_num_threads)
	{
		SimpleLogger().Write(logWARNING) << "The recommended number of threads is "
										 << recommended_num_threads
										 << "! This setting may have performance side-effects.";
	}

	tbb::task_scheduler_init init(requested_num_threads);

	LogPolicy::GetInstance().Unmute();

	FingerPrint fingerprint_orig;
	CheckRestrictionsFile(fingerprint_orig);

	boost::filesystem::ifstream input_stream(input_path, std::ios::in | std::ios::binary);

	node_filename = input_path.string() + ".nodes";
	edge_out = input_path.string() + ".edges";
	geometry_filename = input_path.string() + ".geometry";
	graph_out = input_path.string() + ".hsgr";
	rtree_nodes_path = input_path.string() + ".ramIndex";
	rtree_leafs_path = input_path.string() + ".fileIndex";

	expanded_graph_out = input_path.string() + ".expanded";

	/*** Setup Scripting Environment ***/
	// Create a new lua state
	lua_State *lua_state = luaL_newstate();

	// Connect LuaBind to this lua state
	luabind::open(lua_state);

	EdgeBasedGraphFactory::SpeedProfileProperties speed_profile;

	if (!SetupScriptingEnvironment(lua_state, speed_profile))
	{
		return 1;
	}

#ifdef WIN32
#pragma message("Memory consumption on Windows can be higher due to different bit packing")
#else
	static_assert(sizeof(ImportEdge) == 20,
				  "changing ImportEdge type has influence on memory consumption!");
#endif
	NodeID number_of_node_based_nodes =
			readBinaryOSRMGraphFromStream(input_stream,
										  edge_list,
										  barrier_node_list,
										  traffic_light_list,
										  &internal_to_external_node_map,
										  restriction_list);
	input_stream.close();

	if (edge_list.empty())
	{
		SimpleLogger().Write(logWARNING) << "The input data is empty, exiting.";
		return 1;
	}

	SimpleLogger().Write() << restriction_list.size() << " restrictions, "
						   << barrier_node_list.size() << " bollard nodes, "
						   << traffic_light_list.size() << " traffic lights";

	std::vector<EdgeBasedNode> node_based_edge_list;
	unsigned number_of_edge_based_nodes = 0;
	DeallocatingVector<EdgeBasedEdge> edge_based_edge_list;

	// init node_based_edge_list, edge_based_edge_list by edgeList
	number_of_edge_based_nodes = BuildEdgeExpandedGraph(lua_state,
														number_of_node_based_nodes,
														node_based_edge_list,
														edge_based_edge_list,
														speed_profile);
	lua_close(lua_state);

	TIMER_STOP(expansion);

	BuildRTree(node_based_edge_list);

	RangebasedCRC32 crc32;
	if (crc32.using_hardware())
	{
		SimpleLogger().Write() << "using hardware based CRC32 computation";
	}
	else
	{
		SimpleLogger().Write() << "using software based CRC32 computation";
	}

	const unsigned crc32_value = crc32(node_based_edge_list);
	node_based_edge_list.clear();
	node_based_edge_list.shrink_to_fit();
	SimpleLogger().Write() << "CRC32: " << crc32_value;

	WriteNodeMapping();

	tbb::parallel_sort(edge_based_edge_list.begin(), edge_based_edge_list.end());

	// Storing edges in vector in preparation for preprocessing
	boost::filesystem::ofstream expanded_graph_output_stream(expanded_graph_out, std::ios::binary);
	//check sum
	expanded_graph_output_stream.write((char *)&crc32_value, sizeof(unsigned));
	//number of nodes
	expanded_graph_output_stream.write((char *)&number_of_edge_based_nodes, sizeof(unsigned));
	//number of edges
	unsigned nedges = edge_based_edge_list.size();
	expanded_graph_output_stream.write((char *)&nedges, sizeof(unsigned));
	//serializing edges
	const auto dend = edge_based_edge_list.dend();
	for (auto diter = edge_based_edge_list.dbegin(); diter != dend; ++diter)
	{
		ExpandedEdge tmp_edge;
		tmp_edge.source = diter->source;
		tmp_edge.target = diter->target;
		tmp_edge.distance = static_cast<unsigned int>(std::max(diter->weight, 1));
		tmp_edge.id = diter->edge_id;
		tmp_edge.forward = diter->forward ? true : false;
		tmp_edge.backward = diter->backward ? true : false;
		expanded_graph_output_stream.write((char *)&tmp_edge, sizeof(ExpandedEdge));
	}
	expanded_graph_output_stream.close();

	TIMER_STOP(preparing);

	SimpleLogger().Write() << "Preprocessing : " << TIMER_SEC(preparing) << " seconds";
	SimpleLogger().Write() << "Expansion  : " << (number_of_node_based_nodes / TIMER_SEC(expansion))
						   << " nodes/sec and "
						   << (number_of_edge_based_nodes / TIMER_SEC(expansion)) << " edges/sec";

	SimpleLogger().Write() << "finished preparing";

	return 0;
}

/**
 \brief Parses command line arguments
 \param argc count of arguments
 \param argv array of arguments
 \param result [out] value for exit return value
 \return true if everything is ok, false if need to terminate execution
*/
bool Prepare::ParseArguments(int argc, char *argv[])
{
	// declare a group of options that will be allowed only on command line
	boost::program_options::options_description generic_options("Options");
	generic_options.add_options()("version,v", "Show version")("help,h", "Show this help message")(
				"config,c",
				boost::program_options::value<boost::filesystem::path>(&config_file_path)
				->default_value("contractor.ini"),
				"Path to a configuration file.");

	// declare a group of options that will be allowed both on command line and in config file
	boost::program_options::options_description config_options("Configuration");
	config_options.add_options()(
				"restrictions,r",
				boost::program_options::value<boost::filesystem::path>(&restrictions_path),
				"Restrictions file in .osrm.restrictions format")(
				"profile,p",
				boost::program_options::value<boost::filesystem::path>(&profile_path)
				->default_value("profile.lua"),
				"Path to LUA routing profile")(
				"threads,t",
				boost::program_options::value<unsigned int>(&requested_num_threads)
				->default_value(tbb::task_scheduler_init::default_num_threads()),
				"Number of threads to use");

	// hidden options, will be allowed both on command line and in config file, but will not be
	// shown to the user
	boost::program_options::options_description hidden_options("Hidden options");
	hidden_options.add_options()(
				"input,i",
				boost::program_options::value<boost::filesystem::path>(&input_path),
				"Input file in .osm, .osm.bz2 or .osm.pbf format");

	// positional option
	boost::program_options::positional_options_description positional_options;
	positional_options.add("input", 1);

	// combine above options for parsing
	boost::program_options::options_description cmdline_options;
	cmdline_options.add(generic_options).add(config_options).add(hidden_options);

	boost::program_options::options_description config_file_options;
	config_file_options.add(config_options).add(hidden_options);

	boost::program_options::options_description visible_options(
				"Usage: " + boost::filesystem::basename(argv[0]) + " <input.osrm> [options]");
	visible_options.add(generic_options).add(config_options);

	// parse command line options
	boost::program_options::variables_map option_variables;
	boost::program_options::store(boost::program_options::command_line_parser(argc, argv)
								  .options(cmdline_options)
								  .positional(positional_options)
								  .run(),
								  option_variables);

	const auto& temp_config_path = option_variables["config"].as<boost::filesystem::path>();
	if (boost::filesystem::is_regular_file(temp_config_path))
	{
		boost::program_options::store(boost::program_options::parse_config_file<char>(temp_config_path.string().c_str(), cmdline_options, true),
									  option_variables);
	}

	if (option_variables.count("version"))
	{
		SimpleLogger().Write() << g_GIT_DESCRIPTION;
		return false;
	}

	if (option_variables.count("help"))
	{
		SimpleLogger().Write() << "\n" << visible_options;
		return false;
	}

	boost::program_options::notify(option_variables);

	if (!option_variables.count("restrictions"))
	{
		restrictions_path = std::string(input_path.string() + ".restrictions");
	}

	if (!option_variables.count("input"))
	{
		SimpleLogger().Write() << "\n" << visible_options;
		return false;
	}

	return true;
}

/**
 \brief Loads and checks file UUIDs
*/
void Prepare::CheckRestrictionsFile(FingerPrint &fingerprint_orig)
{
	boost::filesystem::ifstream restriction_stream(restrictions_path, std::ios::binary);
	FingerPrint fingerprint_loaded;
	unsigned number_of_usable_restrictions = 0;
	restriction_stream.read((char *)&fingerprint_loaded, sizeof(FingerPrint));
	if (!fingerprint_loaded.TestPrepare(fingerprint_orig))
	{
		SimpleLogger().Write(logWARNING) << ".restrictions was prepared with different build.\n"
											"Reprocess to get rid of this warning.";
	}

	restriction_stream.read((char *)&number_of_usable_restrictions, sizeof(unsigned));
	restriction_list.resize(number_of_usable_restrictions);
	if (number_of_usable_restrictions > 0)
	{
		restriction_stream.read((char *)&(restriction_list[0]),
				number_of_usable_restrictions * sizeof(TurnRestriction));
	}
	restriction_stream.close();
}

/**
	\brief Setups scripting environment (lua-scripting)
	Also initializes speed profile.
*/
bool
Prepare::SetupScriptingEnvironment(lua_State *lua_state,
								   EdgeBasedGraphFactory::SpeedProfileProperties &speed_profile)
{
	// open utility libraries string library;
	luaL_openlibs(lua_state);

	// adjust lua load path
	luaAddScriptFolderToLoadPath(lua_state, profile_path.string().c_str());

	// Now call our function in a lua script
	if (0 != luaL_dofile(lua_state, profile_path.string().c_str()))
	{
		std::cerr << lua_tostring(lua_state, -1) << " occured in scripting block" << std::endl;
		return false;
	}

	if (0 != luaL_dostring(lua_state, "return traffic_signal_penalty\n"))
	{
		std::cerr << lua_tostring(lua_state, -1) << " occured in scripting block" << std::endl;
		return false;
	}
	speed_profile.traffic_signal_penalty = 10 * lua_tointeger(lua_state, -1);
	SimpleLogger().Write(logDEBUG)
			<< "traffic_signal_penalty: " << speed_profile.traffic_signal_penalty;

	if (0 != luaL_dostring(lua_state, "return u_turn_penalty\n"))
	{
		std::cerr << lua_tostring(lua_state, -1) << " occured in scripting block" << std::endl;
		return false;
	}

	speed_profile.u_turn_penalty = 10 * lua_tointeger(lua_state, -1);
	speed_profile.has_turn_penalty_function = lua_function_exists(lua_state, "turn_function");

	return true;
}

/**
 \brief Building an edge-expanded graph from node-based input and turn restrictions
*/
std::size_t
Prepare::BuildEdgeExpandedGraph(lua_State *lua_state,
								NodeID number_of_node_based_nodes,
								std::vector<EdgeBasedNode> &node_based_edge_list,
								DeallocatingVector<EdgeBasedEdge> &edge_based_edge_list,
								EdgeBasedGraphFactory::SpeedProfileProperties &speed_profile)
{
	SimpleLogger().Write() << "Generating edge-expanded graph representation";
	std::shared_ptr<NodeBasedDynamicGraph> node_based_graph =
			NodeBasedDynamicGraphFromImportEdges(number_of_node_based_nodes, edge_list);
	std::unique_ptr<RestrictionMap> restriction_map = osrm::make_unique<RestrictionMap>(restriction_list);
	std::shared_ptr<EdgeBasedGraphFactory> edge_based_graph_factory =
			std::make_shared<EdgeBasedGraphFactory>(node_based_graph,
													std::move(restriction_map),
													barrier_node_list,
													traffic_light_list,
													internal_to_external_node_map,
													speed_profile);
	edge_list.clear();
	edge_list.shrink_to_fit();

	edge_based_graph_factory->Run(edge_out, geometry_filename, lua_state);

	restriction_list.clear();
	restriction_list.shrink_to_fit();
	barrier_node_list.clear();
	barrier_node_list.shrink_to_fit();
	traffic_light_list.clear();
	traffic_light_list.shrink_to_fit();

	const std::size_t number_of_edge_based_nodes =
			edge_based_graph_factory->GetNumberOfEdgeBasedNodes();

	BOOST_ASSERT(number_of_edge_based_nodes != std::numeric_limits<unsigned>::max());
#ifndef WIN32
	static_assert(sizeof(EdgeBasedEdge) == 16,
				  "changing ImportEdge type has influence on memory consumption!");
#endif

	edge_based_graph_factory->GetEdgeBasedEdges(edge_based_edge_list);
	edge_based_graph_factory->GetEdgeBasedNodes(node_based_edge_list);

	edge_based_graph_factory.reset();
	node_based_graph.reset();

	return number_of_edge_based_nodes;
}

/**
  \brief Writing info on original (node-based) nodes
 */
void Prepare::WriteNodeMapping()
{
	SimpleLogger().Write() << "writing node map ...";
	boost::filesystem::ofstream node_stream(node_filename, std::ios::binary);
	const unsigned size_of_mapping = internal_to_external_node_map.size();
	node_stream.write((char *)&size_of_mapping, sizeof(unsigned));
	if (size_of_mapping > 0)
	{
		node_stream.write((char *)&(internal_to_external_node_map[0]),
				size_of_mapping * sizeof(QueryNode));
	}
	node_stream.close();
	internal_to_external_node_map.clear();
	internal_to_external_node_map.shrink_to_fit();
}

/**
	\brief Building rtree-based nearest-neighbor data structure

	Saves info to files: '.ramIndex' and '.fileIndex'.
 */
void Prepare::BuildRTree(std::vector<EdgeBasedNode> &node_based_edge_list)
{
	SimpleLogger().Write() << "building r-tree ...";
	StaticRTree<EdgeBasedNode>(node_based_edge_list,
							   rtree_nodes_path.c_str(),
							   rtree_leafs_path.c_str(),
							   internal_to_external_node_map);
}
