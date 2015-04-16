#ifndef DSERVER_H
#define DSERVER_H

#include "Connection.h"
#include "RequestHandler.h"

#include "../Util/cast.hpp"
#include "../Util/make_unique.hpp"
#include "../Util/simple_logger.hpp"

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <zlib.h>

#include <functional>
#include <memory>
#include <thread>
#include <vector>

class DynamicServer
{
public:

	// Note: returns a shared instead of a unique ptr as it is captured in a lambda somewhere else
	static std::shared_ptr<DynamicServer> CreateServer(std::string &ip_address, int ip_port, unsigned requested_num_threads)
	{
		SimpleLogger().Write() << "http 1.1 compression handled by zlib version " << zlibVersion();
		const unsigned hardware_threads = std::max(1u, std::thread::hardware_concurrency());
		const unsigned real_num_threads = std::min(hardware_threads, requested_num_threads);
		return std::make_shared<DynamicServer>(ip_address, ip_port, real_num_threads);
	}

	explicit DynamicServer(const std::string &address, const int port, const unsigned thread_pool_size)
		: thread_pool_size(thread_pool_size), acceptor(io_service),
		  new_connection(std::make_shared<http::Connection>(io_service, request_handler)), request_handler()
	{
		const std::string port_string = cast::integral_to_string(port);

		boost::asio::ip::tcp::resolver resolver(io_service);
		/*
		Fix here : http://stackoverflow.com/questions/12542460/boost-asio-host-not-found-authorative
		 * The problem was that the constructor for query has the address_configured flag set by default
		 *  which won't return an address if the loopback device is the only device with an address.
		 * By just settings flags to 0 or anything other than address_configured the problem is fixed.
		boost::asio::ip::tcp::resolver::query query(address, port_string);
		*/
		boost::asio::ip::tcp::resolver::query query(address, port_string, boost::asio::ip::resolver_query_base::numeric_service);
		boost::asio::ip::tcp::endpoint endpoint = *resolver.resolve(query);

		acceptor.open(endpoint.protocol());
		acceptor.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));
		acceptor.bind(endpoint);
		acceptor.listen();
		acceptor.async_accept(
					new_connection->socket(),
					boost::bind(&DynamicServer::HandleAccept, this, boost::asio::placeholders::error));
	}

	void Run()
	{
		std::vector<std::shared_ptr<std::thread>> threads;
		for (unsigned i = 0; i < thread_pool_size; ++i)
		{
			std::shared_ptr<std::thread> thread = std::make_shared<std::thread>(
						boost::bind(&boost::asio::io_service::run, &io_service));
			threads.push_back(thread);
		}
		for (auto thread : threads)
		{
			thread->join();
		}
	}

	void Stop() { io_service.stop(); }

	RequestHandler &GetRequestHandlerPtr() { return request_handler; }

private:
	void HandleAccept(const boost::system::error_code &e)
	{
		if (!e)
		{
			new_connection->start();
			new_connection = std::make_shared<http::Connection>(io_service, request_handler);
			acceptor.async_accept(
						new_connection->socket(),
						boost::bind(&DynamicServer::HandleAccept, this, boost::asio::placeholders::error));
		}
	}

	unsigned thread_pool_size;
	boost::asio::io_service io_service;
	boost::asio::ip::tcp::acceptor acceptor;
	std::shared_ptr<http::Connection> new_connection;
	RequestHandler request_handler;
};

#endif // DSERVER_H
