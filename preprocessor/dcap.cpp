/*

Copyright (c) 2015, Project DevacuS, Mohamed Neggaz, others
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

#include "dcap.hpp"

#include "../expander/processing_chain.hpp"

#include "../expander/contractor.hpp"

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


DCAPPreprocess::DCAPPreprocess() : requested_num_threads(1) {}

DCAPPreprocess::~DCAPPreprocess() {}

int DCAPPreprocess::Run(int argc, char *argv[])
{
	LogPolicy::GetInstance().Unmute();

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

	node_filename = input_path.string() + ".nodes";
	edge_out = input_path.string() + ".edges";
	geometry_filename = input_path.string() + ".geometry";
	graph_out = input_path.string() + ".hsgr";
	rtree_nodes_path = input_path.string() + ".ramIndex";
	rtree_leafs_path = input_path.string() + ".fileIndex";

	expanded_graph_out = input_path.string() + ".expanded";


	//Restoring edge-expanded graph from file
	boost::filesystem::ifstream expanded_graph_stream(expanded_graph_out, std::ios::in | std::ios::binary);

	//std::vector<EdgeBasedNode> node_based_edge_list;
	unsigned number_of_edge_based_nodes = 0;
	unsigned number_of_edge_based_edges = 0;
	unsigned crc32_value;
	DeallocatingVector<EdgeBasedEdge> restored_edge_based_edge_list;
	restored_edge_based_edge_list.clear();

	//reading check sum
	expanded_graph_stream.read((char *)&crc32_value, sizeof(unsigned));
	//raeding input file
	expanded_graph_stream.read((char *)&number_of_edge_based_nodes, sizeof(unsigned));
	//number of edges
	expanded_graph_stream.read((char *)&number_of_edge_based_edges, sizeof(unsigned));
	//reading edges
	ExpandedEdge tmp_edge;
	for ( unsigned i = 0; i < number_of_edge_based_edges ; ++ i )
	{
		expanded_graph_stream.read((char *)&tmp_edge, sizeof(ExpandedEdge));
		restored_edge_based_edge_list.emplace_back(EdgeBasedEdge(tmp_edge.source,
														  tmp_edge.target,
														  tmp_edge.id,
														  tmp_edge.distance,
														  tmp_edge.forward,
														  tmp_edge.backward));
	}
	expanded_graph_stream.close();

	/***
	 * Partitioning graph
	 */



	/***
	 * Preprocessing data
	 */
	return 0;
}

bool DCAPPreprocess::ParseArguments(int argc, char *argv[])
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

void DCAPPreprocess::CheckRestrictionsFile(FingerPrint &fingerprint_orig)
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


