namespace boost { namespace interprocess { class named_mutex; } }

#include "DRM_impl.h"
#include "DRM.h"

#include <osrm/Reply.h>
#include <osrm/RouteParameters.h>
#include <osrm/ServerPaths.h>

/*
#include "../plugins/distance_table.hpp"
#include "../plugins/hello_world.hpp"
#include "../plugins/locate.hpp"
#include "../plugins/nearest.hpp"
#include "../plugins/timestamp.hpp"
#include "../plugins/viaroute.hpp"
//*/

#include "../plugins/hello_world.hpp"
#include "../plugins/nodeid.hpp"
#include "../plugins/baseroute.hpp"

#include "../Server/DataStructures/BaseDataFacade.h"
#include "../Server/DataStructures/InternalDataFacade.h"
#include "../Server/DataStructures/SharedBarriers.h"
#include "../Server/DataStructures/SharedDataFacade.h"
#include "../Util/make_unique.hpp"
#include "../Util/ProgramOptions.h"
#include "../Util/simple_logger.hpp"

#include <boost/assert.hpp>
#include <boost/interprocess/sync/named_condition.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <algorithm>
#include <fstream>
#include <utility>
#include <vector>

DRM_impl::DRM_impl(ServerPaths server_paths)
{
	// populate base path
	populate_base_path(server_paths);
	query_data_facade = new InternalDataFacade<QueryEdge::EdgeData>(server_paths);

	/*
	// The following plugins handle all requests.
	RegisterPlugin(new DistanceTablePlugin<BaseDataFacade<QueryEdge::EdgeData>>(query_data_facade));
	RegisterPlugin(new HelloWorldPlugin());
	RegisterPlugin(new LocatePlugin<BaseDataFacade<QueryEdge::EdgeData>>(query_data_facade));
	RegisterPlugin(new NearestPlugin<BaseDataFacade<QueryEdge::EdgeData>>(query_data_facade));
	RegisterPlugin(new TimestampPlugin<BaseDataFacade<QueryEdge::EdgeData>>(query_data_facade));
	RegisterPlugin(new ViaRoutePlugin<BaseDataFacade<QueryEdge::EdgeData>>(query_data_facade));
	*/


	RegisterPlugin(new HelloWorldPlugin());
	RegisterPlugin(new NodeIDPlugin<QueryEdge::EdgeData>(query_data_facade));
	RegisterPlugin(new BaseRoutePlugin<QueryEdge::EdgeData>(query_data_facade));
}

DRM_impl::~DRM_impl()
{
	delete query_data_facade;
	for (PluginMap::value_type &plugin_pointer : plugin_map)
	{
		delete plugin_pointer.second;
	}
}

void DRM_impl::RegisterPlugin(BasePlugin *plugin)
{
	SimpleLogger().Write() << "loaded plugin: " << plugin->GetDescriptor();
	if (plugin_map.find(plugin->GetDescriptor()) != plugin_map.end())
	{
		delete plugin_map.find(plugin->GetDescriptor())->second;
	}
	plugin_map.emplace(plugin->GetDescriptor(), plugin);
}

void DRM_impl::RunQuery(RouteParameters &route_parameters, http::Reply &reply)
{
	const PluginMap::const_iterator &iter = plugin_map.find(route_parameters.service);

	if (plugin_map.end() != iter)
	{
		reply.status = http::Reply::ok;
		iter->second->HandleRequest(route_parameters, reply);
	}
	else
	{
		reply = http::Reply::StockReply(http::Reply::badRequest);
	}
}

// proxy code for compilation firewall

DRM::DRM(ServerPaths paths)
	: DRM_pimpl_(osrm::make_unique<DRM_impl>(paths))
{
}

DRM::~DRM() { DRM_pimpl_.reset(); }

void DRM::RunQuery(RouteParameters &route_parameters, http::Reply &reply)
{
	DRM_pimpl_->RunQuery(route_parameters, reply);
}
