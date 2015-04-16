#ifndef DRM_SEARCH_ENGINE_HPP
#define DRM_SEARCH_ENGINE_HPP

#include "search_engine_data.hpp"
#include "../DynamicServer/DataStructures/InternalDataFacade.h"

#include "../routing_algorithms/dijkstra.hpp"

#include <type_traits>

template <class EdgeDataT> class DRMSearchEngine
{
private:
	InternalDataFacade<EdgeDataT> *facade;
	SearchEngineData engine_working_data;

public:
	BasicDijkstraRouting<InternalDataFacade<EdgeDataT>> dijkstra_path;

	explicit DRMSearchEngine(InternalDataFacade<EdgeDataT> *facade)
		: facade(facade), dijkstra_path(facade, engine_working_data)
	{
		static_assert(!std::is_pointer<EdgeDataT>::value, "don't instantiate with ptr type");
		static_assert(std::is_object<EdgeDataT>::value, "don't instantiate with void, function, or reference");
	}

	~DRMSearchEngine() {}
};

#endif // DRM_SEARCH_ENGINE_HPP
