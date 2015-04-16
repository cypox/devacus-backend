#ifndef DCAP_HPP
#define DCAP_HPP

#include "preprocess.hpp"

#include "../expander/edge_based_graph_factory.hpp"
#include "../data_structures/query_edge.hpp"
#include "../data_structures/static_graph.hpp"

class FingerPrint;
struct EdgeBasedNode;
struct lua_State;

#include <boost/filesystem.hpp>

#include <vector>

class DCAPPreprocess : Preprocess
{
	struct ExpandedEdge
	{
		ExpandedEdge()
			: source(0), target(0), id(0), distance(0), forward(0), backward(0)
		{
		}
		ExpandedEdge(unsigned source,
					  unsigned target,
					  unsigned id,
					  unsigned distance,
					  bool forward,
					  bool backward)
			: source(source), target(target), id(id), distance(distance), forward(forward), backward(backward)
		{
		}
		unsigned source;
		unsigned target;
		unsigned id;
		unsigned distance;
		bool forward : 1;
		bool backward : 1;
	};
	struct EdgeContainer
	{
		EdgeContainer()
			: source(0), target(0), distance(0), id(0), originalEdges(0), shortcut(0), forward(0), backward(0),
			  is_original_via_node_ID(false)
		{
		}
		EdgeContainer(unsigned source,
					  unsigned target,
					  unsigned distance,
					  unsigned original_edges,
					  unsigned id,
					  bool shortcut,
					  bool forward,
					  bool backward)
			: source(source), target(target), distance(distance), id(id),
			  originalEdges(std::min((unsigned)1 << 28, original_edges)), shortcut(shortcut),
			  forward(forward), backward(backward), is_original_via_node_ID(false)
		{
		}
		unsigned source;
		unsigned target;
		unsigned distance;
		unsigned id;
		unsigned originalEdges : 28;
		bool shortcut : 1;
		bool forward : 1;
		bool backward : 1;
		bool is_original_via_node_ID : 1;
	};
public:
	using EdgeData = QueryEdge::EdgeData;
	using InputEdge = DynamicGraph<EdgeData>::InputEdge;
	using StaticEdge = StaticGraph<EdgeData>::InputEdge;

	explicit DCAPPreprocess();
	DCAPPreprocess(const DCAPPreprocess &) = delete;
	~DCAPPreprocess();

	int Run(int argc, char *argv[]);

protected:
	bool ParseArguments(int argc, char *argv[]);
	void CheckRestrictionsFile(FingerPrint &fingerprint_orig);

private:
	std::vector<QueryNode> internal_to_external_node_map;
	std::vector<TurnRestriction> restriction_list;
	std::vector<NodeID> barrier_node_list;
	std::vector<NodeID> traffic_light_list;
	std::vector<ImportEdge> edge_list;

	unsigned requested_num_threads;
	boost::filesystem::path config_file_path;
	boost::filesystem::path input_path;
	boost::filesystem::path restrictions_path;
	boost::filesystem::path preinfo_path;
	boost::filesystem::path profile_path;

	std::string node_filename;
	std::string edge_out;
	std::string info_out;
	std::string geometry_filename;
	std::string graph_out;
	std::string rtree_nodes_path;
	std::string rtree_leafs_path;

	std::string expanded_graph_out;
};

#endif // DCAP_HPP
