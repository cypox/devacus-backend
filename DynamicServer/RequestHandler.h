#ifndef REQUEST_HANDLER_H
#define REQUEST_HANDLER_H

#include "../LibDRM/DRM.h"

#include <string>

template <typename Iterator, class HandlerT> struct APIGrammar;
struct RouteParameters;
class OSRM;

namespace http
{
class Reply;
struct Request;
}

class RequestHandler
{

public:
	using APIGrammarParser = APIGrammar<std::string::iterator, RouteParameters>;

	RequestHandler();
	RequestHandler(const RequestHandler &) = delete;

	void handle_request(const http::Request &req, http::Reply &rep);
	void RegisterRoutingMachine(DRM *drm);

private:
	DRM *d_routing_machine;
};

#endif // REQUEST_HANDLER_H
