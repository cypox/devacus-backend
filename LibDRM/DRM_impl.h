#ifndef DRM_IMPL_H
#define DRM_IMPL_H

class BasePlugin;
namespace http { class Reply; }
struct RouteParameters;

#include <osrm/ServerPaths.h>

#include "../data_structures/query_edge.hpp"
#include "../DynamicServer/DataStructures/InternalDataFacade.h"

#include <memory>
#include <unordered_map>
#include <string>

//template <class EdgeDataT> class BaseDataFacade;

class DRM_impl
{
private:
	using PluginMap = std::unordered_map<std::string, BasePlugin *>;

public:
	DRM_impl(ServerPaths paths);
	DRM_impl(const DRM_impl &) = delete;
	virtual ~DRM_impl();
	void RunQuery(RouteParameters &route_parameters, http::Reply &reply);

private:
	void RegisterPlugin(BasePlugin *plugin);
	PluginMap plugin_map;
	// base class pointer to the objects
	InternalDataFacade<QueryEdge::EdgeData> *query_data_facade;
};

#endif // DRM_IMPL_H
