#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

#include <boost/assert.hpp>

#include "routing_base.hpp"
#include "../DynamicServer/DataStructures/InternalDataFacade.h"
#include "../data_structures/search_engine_data.hpp"
#include "../Util/integer_range.hpp"
#include "../Util/timing_util.hpp"
#include "../typedefs.h"

template <class DataFacadeT> class BasicDijkstraRouting final : public BasicRoutingInterface<DataFacadeT>
{
	typedef typename DataFacadeT::EdgeData EdgeData;

	using super = BasicRoutingInterface<DataFacadeT>;
	using QueryHeap = SearchEngineData::QueryHeap;
	SearchEngineData &engine_working_data;

public:
	BasicDijkstraRouting(DataFacadeT *facade, SearchEngineData &engine_working_data)
		: super(facade), engine_working_data(engine_working_data)
	{
	}

	~BasicDijkstraRouting() {}

	void operator()(const std::vector<PhantomNodes> &phantom_nodes_vector,
					const std::vector<bool> &uturn_indicators,
					RawRouteData &raw_route_data) const
	{
		TIMER_START(query);

		int distance = 0;
		int setteled_nodes = 0;
		std::vector<NodeID> path;

		QueryHeap dijkstra_heap(super::facade->GetNumberOfNodes());
		NodeID source = phantom_nodes_vector[0].source_phantom.forward_node_id;
		NodeID target = SPECIAL_NODEID;
		NodeID f_target = phantom_nodes_vector[0].target_phantom.forward_node_id;
		NodeID r_target = phantom_nodes_vector[0].target_phantom.reverse_node_id;
		NodeID current = SPECIAL_NODEID;

		dijkstra_heap.Clear();

		const bool allow_u_turn = false;

		if (source != SPECIAL_NODEID)
		{
			dijkstra_heap.Insert(
						phantom_nodes_vector[0].source_phantom.forward_node_id,
					(allow_u_turn ? 0 : distance) - phantom_nodes_vector[0].source_phantom.GetForwardWeightPlusOffset(),
					phantom_nodes_vector[0].source_phantom.forward_node_id);
			dijkstra_heap.Insert(
						phantom_nodes_vector[0].source_phantom.reverse_node_id,
					(allow_u_turn ? 0 : distance) - phantom_nodes_vector[0].source_phantom.GetReverseWeightPlusOffset(),
					phantom_nodes_vector[0].source_phantom.reverse_node_id);

			while ( !dijkstra_heap.Empty() )
			{
				current = dijkstra_heap.DeleteMin();
				distance = dijkstra_heap.GetKey(current);

				//SimpleLogger().Write(logDEBUG) << "expanding node " << current;

				if ( current == f_target || current == r_target )
				{
					target = current;
					SimpleLogger().Write(logDEBUG) << "found road to target " << current << " of distance " << distance << std::endl
												   << "setteled nodes " << setteled_nodes;
					break;
				}

				for (const auto edge : super::facade->GetAdjacentEdgeRange(current))
				{
					const EdgeData &data = super::facade->GetEdgeData(edge);
					const int edge_weight = data.distance;
					BOOST_ASSERT_MSG(edge_weight > 0, "edge_weight invalid");

					if ( data.forward )
					{
						// New Node discovered -> Add to Heap + Node Info Storage
						const NodeID to = super::facade->GetTarget(edge);
						const int to_distance = distance + edge_weight;
						if (!dijkstra_heap.WasInserted(to))
						{
							dijkstra_heap.Insert(to, to_distance, current);
							//SimpleLogger().Write(logDEBUG) << "discovering node " << to << " with distance " << to_distance;
						}
						// Found a shorter Path -> Update distance
						else if (to_distance < dijkstra_heap.GetKey(to))
						{
							// new parent
							dijkstra_heap.GetData(to).parent = current;
							dijkstra_heap.DecreaseKey(to, to_distance);
							//SimpleLogger().Write(logDEBUG) << "improving road to " << to << " with " << to_distance;
						}
					}
				}
				++setteled_nodes;
			}
		}

		BOOST_ASSERT_MSG(target != SPECIAL_NODEID, "edge_weight invalid");
		// Did we found anything ?
		if ( current != target )
		{
			raw_route_data.shortest_path_length = INVALID_EDGE_WEIGHT;
			return;
		}

		NodeID node = target;
		while (node != dijkstra_heap.GetData(node).parent)
		{
			path.emplace_back(node);
			node = dijkstra_heap.GetData(node).parent;
		}
		path.emplace_back(node);

		std::reverse(path.begin(), path.end());

		raw_route_data.unpacked_path_segments.resize(phantom_nodes_vector.size());
		super::UnpackPath(
					// -- packed input
					path,
					// -- start and end of route
					phantom_nodes_vector[0],
				// -- unpacked output
				raw_route_data.unpacked_path_segments[0]);

		raw_route_data.source_traversed_in_reverse.push_back(
					(path.front() != phantom_nodes_vector[0].source_phantom.forward_node_id));
		raw_route_data.target_traversed_in_reverse.push_back(
					(path.back() != phantom_nodes_vector[0].target_phantom.forward_node_id));
		raw_route_data.shortest_path_length = ( dijkstra_heap.GetKey(target) < 0 ) ? 0 :
																				  dijkstra_heap.GetKey(target);

		TIMER_STOP(query);
		SimpleLogger().Write() << "Qeury : " << TIMER_SEC(query) << " seconds";
	}
};

#endif /* DIJKSTRA_HPP */
