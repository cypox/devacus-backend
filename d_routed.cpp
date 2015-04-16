#include "LibDRM/DRM.h"
#include "DynamicServer/DynamicServer.h"
#include "Util/git_sha.hpp"
#include "Util/ProgramOptions.h"
#include "Util/simple_logger.hpp"

#include <sys/mman.h>

#include <cstdlib>

#include <signal.h>

#include <chrono>
#include <future>
#include <iostream>
#include <thread>

int main(int argc, const char *argv[])
{
	try
	{
		LogPolicy::GetInstance().Unmute();

		bool use_shared_memory = false, trial_run = false;
		std::string ip_address;
		int ip_port, requested_thread_num;

		ServerPaths server_paths;

		const unsigned init_result = GenerateServerProgramOptions(argc,
																  argv,
																  server_paths,
																  ip_address,
																  ip_port,
																  requested_thread_num,
																  use_shared_memory,
																  trial_run);
		if (init_result == INIT_OK_DO_NOT_START_ENGINE)
		{
			return 0;
		}
		if (init_result == INIT_FAILED)
		{
			return 1;
		}

		const int lock_flags = MCL_CURRENT | MCL_FUTURE;
		if (-1 == mlockall(lock_flags))
		{
			SimpleLogger().Write(logWARNING) << argv[0] << " could not be locked to RAM";
		}
		SimpleLogger().Write() << "starting up engines, " << g_GIT_DESCRIPTION;

		if (use_shared_memory)
		{
			SimpleLogger().Write(logDEBUG) << "Loading from shared memory";
		}

		SimpleLogger().Write(logDEBUG) << "Threads:\t" << requested_thread_num;
		SimpleLogger().Write(logDEBUG) << "IP address:\t" << ip_address;
		SimpleLogger().Write(logDEBUG) << "IP port:\t" << ip_port;
		int sig = 0;
		sigset_t new_mask;
		sigset_t old_mask;
		sigfillset(&new_mask);
		pthread_sigmask(SIG_BLOCK, &new_mask, &old_mask);

		DRM drm_lib(server_paths);
		auto routing_server =
				DynamicServer::CreateServer(ip_address, ip_port, requested_thread_num);

		routing_server->GetRequestHandlerPtr().RegisterRoutingMachine(&drm_lib);

		if (trial_run)
		{
			SimpleLogger().Write() << "trial run, quitting after successful initialization";
		}
		else
		{
			std::packaged_task<int()> server_task([&]()->int{ routing_server->Run(); return 0; });
			auto future = server_task.get_future();
			std::thread server_thread(std::move(server_task));

			sigset_t wait_mask;
			pthread_sigmask(SIG_SETMASK, &old_mask, 0);
			sigemptyset(&wait_mask);
			sigaddset(&wait_mask, SIGINT);
			sigaddset(&wait_mask, SIGQUIT);
			sigaddset(&wait_mask, SIGTERM);
			pthread_sigmask(SIG_BLOCK, &wait_mask, 0);
			SimpleLogger().Write() << "running and waiting for requests";
			sigwait(&wait_mask, &sig);

			SimpleLogger().Write() << "initiating shutdown";
			routing_server->Stop();
			SimpleLogger().Write() << "stopping threads";

			auto status = future.wait_for(std::chrono::seconds(2));

			if (status == std::future_status::ready)
			{
				server_thread.join();
			}
			else
			{
				SimpleLogger().Write(logWARNING) << "Didn't exit within 2 seconds. Hard abort!";
				server_task.reset(); // just kill it
			}
		}

		SimpleLogger().Write() << "freeing objects";
		routing_server.reset();
		SimpleLogger().Write() << "shutdown completed";
	}
	catch (const std::exception &e)
	{
		SimpleLogger().Write(logWARNING) << "exception: " << e.what();
		return 1;
	}

	munlockall();

	return 0;
}
