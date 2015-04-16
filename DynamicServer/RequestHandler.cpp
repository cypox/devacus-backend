#include "RequestHandler.h"

#include "APIGrammar.h"
#include "../Server/Http/Request.h"

#include "../data_structures/json_container.hpp"
#include "../LibDRM/DRM.h"
#include "../Util/json_renderer.hpp"
#include "../Util/simple_logger.hpp"
#include "../Util/string_util.hpp"
#include "../typedefs.h"

#include <osrm/Reply.h>
#include <osrm/RouteParameters.h>

#include <ctime>

#include <algorithm>
#include <iostream>

RequestHandler::RequestHandler() : d_routing_machine(nullptr) {}

void RequestHandler::handle_request(const http::Request &req, http::Reply &reply)
{
	// parse command
	try
	{
		std::string request;
		URIDecode(req.uri, request);

		// deactivated as GCC apparently does not implement that, not even in 4.9
		// std::time_t t = std::time(nullptr);
		// SimpleLogger().Write() << std::put_time(std::localtime(&t), "%m-%d-%Y %H:%M:%S") <<
		//     " " << req.endpoint.to_string() << " " <<
		//     req.referrer << ( 0 == req.referrer.length() ? "- " :" ") <<
		//     req.agent << ( 0 == req.agent.length() ? "- " :" ") << request;

		time_t ltime;
		struct tm *time_stamp;

		ltime = time(nullptr);
		time_stamp = localtime(&ltime);

		// log timestamp
		SimpleLogger().Write() << (time_stamp->tm_mday < 10 ? "0" : "") << time_stamp->tm_mday << "-"
							   << (time_stamp->tm_mon + 1 < 10 ? "0" : "") << (time_stamp->tm_mon + 1) << "-"
							   << 1900 + time_stamp->tm_year << " " << (time_stamp->tm_hour < 10 ? "0" : "")
							   << time_stamp->tm_hour << ":" << (time_stamp->tm_min < 10 ? "0" : "") << time_stamp->tm_min
							   << ":" << (time_stamp->tm_sec < 10 ? "0" : "") << time_stamp->tm_sec << " "
							   << req.endpoint.to_string() << " " << req.referrer
							   << (0 == req.referrer.length() ? "- " : " ") << req.agent
							   << (0 == req.agent.length() ? "- " : " ") << request;

		RouteParameters route_parameters;
		APIGrammarParser api_parser(&route_parameters);

		auto iter = request.begin();
		const bool result = boost::spirit::qi::parse(iter, request.end(), api_parser);

		// check if the was an error with the request
		if (!result || (iter != request.end()))
		{
			reply = http::Reply::StockReply(http::Reply::badRequest);
			reply.content.clear();
			const auto position = std::distance(request.begin(), iter);
			JSON::Object json_result;
			json_result.values["status"] = 400;
			std::string message = "Query string malformed close to position ";
			message += cast::integral_to_string(position);
			json_result.values["status_message"] = message;
			JSON::render(reply.content, json_result);
			return;
		}

		// parsing done, lets call the right plugin to handle the request
		BOOST_ASSERT_MSG(d_routing_machine != nullptr, "pointer not init'ed");

		if (!route_parameters.jsonp_parameter.empty())
		{ // prepend response with jsonp parameter
			const std::string json_p = (route_parameters.jsonp_parameter + "(");
			reply.content.insert(reply.content.end(), json_p.begin(), json_p.end());
		}
		d_routing_machine->RunQuery(route_parameters, reply);
		if (!route_parameters.jsonp_parameter.empty())
		{ // append brace to jsonp response
			reply.content.push_back(')');
		}

		// set headers
		reply.headers.emplace_back("Content-Length", cast::integral_to_string(reply.content.size()));
		if ("gpx" == route_parameters.output_format)
		{ // gpx file
			reply.headers.emplace_back("Content-Type", "application/gpx+xml; charset=UTF-8");
			reply.headers.emplace_back("Content-Disposition", "attachment; filename=\"route.gpx\"");
		}
		else if (route_parameters.jsonp_parameter.empty())
		{ // json file
			reply.headers.emplace_back("Content-Type", "application/json; charset=UTF-8");
			reply.headers.emplace_back("Content-Disposition", "inline; filename=\"response.json\"");
		}
		else
		{ // jsonp
			reply.headers.emplace_back("Content-Type", "text/javascript; charset=UTF-8");
			reply.headers.emplace_back("Content-Disposition", "inline; filename=\"response.js\"");
		}
	}
	catch (const std::exception &e)
	{
		reply = http::Reply::StockReply(http::Reply::internalServerError);
		SimpleLogger().Write(logWARNING) << "[server error] code: " << e.what()
										 << ", uri: " << req.uri;
		return;
	}
}

void RequestHandler::RegisterRoutingMachine(DRM *drm) { d_routing_machine = drm; }
