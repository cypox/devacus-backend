#ifndef BASE_ROUTE_HPP
#define BASE_ROUTE_HPP

#include "plugin_base.hpp"

#include "../algorithms/object_encoder.hpp"
#include "../data_structures/search_engine.hpp"
#include "../descriptors/descriptor_base.hpp"
#include "../descriptors/gpx_descriptor.hpp"
#include "../descriptors/json_descriptor.hpp"
#include "../Util/integer_range.hpp"
#include "../Util/json_renderer.hpp"
#include "../Util/make_unique.hpp"
#include "../Util/simple_logger.hpp"
#include "../data_structures/drm_search_engine.hpp"
#include "../DynamicServer/DataStructures/InternalDataFacade.h"

#include <cstdlib>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

template <class EdgeDataT> class BaseRoutePlugin final : public BasePlugin
{
private:
	DescriptorTable descriptor_table;
	std::string descriptor_string;
	std::unique_ptr<DRMSearchEngine<EdgeDataT>> search_engine_ptr;
	InternalDataFacade<EdgeDataT> *facade;

public:
	explicit BaseRoutePlugin(InternalDataFacade<EdgeDataT> *facade) : descriptor_string("baseroute"), facade(facade)
	{
		search_engine_ptr = osrm::make_unique<DRMSearchEngine<EdgeDataT>>(facade);
		descriptor_table.emplace("json", 0);
	}

	virtual ~BaseRoutePlugin() {}

	const std::string GetDescriptor() const final { return descriptor_string; }

	void HandleRequest(const RouteParameters &route_parameters, http::Reply &reply) final
	{
		if (2 != route_parameters.coordinates.size() ||
				!route_parameters.coordinates[0].is_valid() ||
				!route_parameters.coordinates[1].is_valid())
		{
			reply = http::Reply::StockReply(http::Reply::badRequest);
			return;
		}
		reply.status = http::Reply::ok;

		PhantomNode source, target;
		facade->IncrementalFindPhantomNodeForCoordinate(route_parameters.coordinates[0], source);
		facade->IncrementalFindPhantomNodeForCoordinate(route_parameters.coordinates[1], target);

		RawRouteData raw_route;
		raw_route.segment_end_coordinates.emplace_back(PhantomNodes{source, target});

		search_engine_ptr->dijkstra_path(raw_route.segment_end_coordinates,
										 route_parameters.uturns,
										 raw_route);

		if (INVALID_EDGE_WEIGHT == raw_route.shortest_path_length)
		{
			SimpleLogger().Write(logDEBUG) << "Error occurred, single path not found";
		}

		std::unique_ptr<BaseDescriptor<InternalDataFacade<EdgeDataT>>> descriptor;
		descriptor = osrm::make_unique<JSONDescriptor<InternalDataFacade<EdgeDataT>>>(facade);
		descriptor->SetConfig(route_parameters);
		descriptor->Run(raw_route, reply);
	}
};

#endif // BASE_ROUTE_HPP
