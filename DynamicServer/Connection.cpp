#include "Connection.h"
#include "RequestHandler.h"
#include "RequestParser.h"

#include <boost/assert.hpp>
#include <boost/bind.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include <string>
#include <vector>

namespace http
{

Connection::Connection(boost::asio::io_service &io_service, RequestHandler &handler)
	: strand(io_service), TCP_socket(io_service), request_handler(handler)
{
}

boost::asio::ip::tcp::socket &Connection::socket() { return TCP_socket; }

/// Start the first asynchronous operation for the connection.
void Connection::start()
{
	TCP_socket.async_read_some(
				boost::asio::buffer(incoming_data_buffer),
				strand.wrap(boost::bind(&Connection::handle_read,
										this->shared_from_this(),
										boost::asio::placeholders::error,
										boost::asio::placeholders::bytes_transferred)));
}

void Connection::handle_read(const boost::system::error_code &error, std::size_t bytes_transferred)
{
	if (error)
	{
		return;
	}

	// no error detected, let's parse the request
	CompressionType compression_type(noCompression);
	boost::tribool result;
	boost::tie(result, boost::tuples::ignore) =
			RequestParser().Parse(request,
								  incoming_data_buffer.data(),
								  incoming_data_buffer.data() + bytes_transferred,
								  compression_type);

	// the request has been parsed
	if (result)
	{
		request.endpoint = TCP_socket.remote_endpoint().address();
		request_handler.handle_request(request, reply);

		// Header compression_header;
		std::vector<char> compressed_output;
		std::vector<boost::asio::const_buffer> output_buffer;

		// compress the result w/ gzip/deflate if requested
		switch (compression_type)
		{
		case deflateRFC1951:
			// use deflate for compression
			reply.headers.insert(reply.headers.begin(), {"Content-Encoding", "deflate"});
			CompressBufferCollection(reply.content, compression_type, compressed_output);
			reply.SetSize(static_cast<unsigned>(compressed_output.size()));
			output_buffer = reply.HeaderstoBuffers();
			output_buffer.push_back(boost::asio::buffer(compressed_output));
			break;
		case gzipRFC1952:
			// use gzip for compression
			reply.headers.insert(reply.headers.begin(), {"Content-Encoding", "gzip"});
			CompressBufferCollection(reply.content, compression_type, compressed_output);
			reply.SetSize(static_cast<unsigned>(compressed_output.size()));
			output_buffer = reply.HeaderstoBuffers();
			output_buffer.push_back(boost::asio::buffer(compressed_output));
			break;
		case noCompression:
			// don't use any compression
			reply.SetUncompressedSize();
			output_buffer = reply.ToBuffers();
			break;
		}
		// write result to stream
		boost::asio::async_write(TCP_socket,
								 output_buffer,
								 strand.wrap(boost::bind(&Connection::handle_write,
														 this->shared_from_this(),
														 boost::asio::placeholders::error)));
	}
	else if (!result)
	{ // request is not parseable
		reply = Reply::StockReply(Reply::badRequest);

		boost::asio::async_write(TCP_socket,
								 reply.ToBuffers(),
								 strand.wrap(boost::bind(&Connection::handle_write,
														 this->shared_from_this(),
														 boost::asio::placeholders::error)));
	}
	else
	{
		// we don't have a result yet, so continue reading
		TCP_socket.async_read_some(
					boost::asio::buffer(incoming_data_buffer),
					strand.wrap(boost::bind(&Connection::handle_read,
											this->shared_from_this(),
											boost::asio::placeholders::error,
											boost::asio::placeholders::bytes_transferred)));
	}
}

/// Handle completion of a write operation.
void Connection::handle_write(const boost::system::error_code &error)
{
	if (!error)
	{
		// Initiate graceful connection closure.
		boost::system::error_code ignore_error;
		TCP_socket.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ignore_error);
	}
}

void Connection::CompressBufferCollection(std::vector<char> uncompressed_data,
										  CompressionType compression_type,
										  std::vector<char> &compressed_data)
{
	boost::iostreams::gzip_params compression_parameters;

	// there's a trade-off between speed and size. speed wins
	compression_parameters.level = boost::iostreams::zlib::best_speed;
	// check which compression flavor is used
	if (deflateRFC1951 == compression_type)
	{
		compression_parameters.noheader = true;
	}

	BOOST_ASSERT(compressed_data.empty());
	// plug data into boost's compression stream
	boost::iostreams::filtering_ostream gzip_stream;
	gzip_stream.push(boost::iostreams::gzip_compressor(compression_parameters));
	gzip_stream.push(boost::iostreams::back_inserter(compressed_data));
	gzip_stream.write(&uncompressed_data[0], uncompressed_data.size());
	boost::iostreams::close(gzip_stream);
}
}
