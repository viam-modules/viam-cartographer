// This is an experimental integration of cartographer into RDK.
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server_builder.h>
#include <signal.h>

#include <iostream>
#include <thread>

#include "glog/logging.h"
#include "slam_service/config.h"
#include "slam_service/slam_service.h"

void exit_loop_handler(int s) {
    LOG(INFO) << "Finishing session.";
    viam::b_continue_session = false;
}

int main(int argc, char** argv) {

    LOG(INFO) << "Server222 \n";
    // glog only supports logging to files and stderr, not stdout.
    FLAGS_logtostderr = 1;
    google::InitGoogleLogging(argv[0]);
    struct sigaction sigHandler;

    sigHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigHandler.sa_mask);
    sigHandler.sa_flags = 0;

    sigaction(SIGTERM, &sigHandler, NULL);
    sigaction(SIGINT, &sigHandler, NULL);

    static_assert((sizeof(float) == 4) && (CHAR_BIT == 8) && (sizeof(int) == 4),
                  "32 bit float & 8 bit char & 32 bit int is assumed");

    viam::SLAMServiceImpl slamService;
    viam::config::ParseAndValidateConfigParams(argc, argv, slamService);

    // Setup the SLAM gRPC server
    grpc::ServerBuilder builder;

    std::unique_ptr<int> selected_port = std::make_unique<int>(0);
    builder.AddListeningPort(slamService.port,
                             grpc::InsecureServerCredentials(),
                             selected_port.get());
    // Increasing the gRPC max message size from the default value of 4MB to
    // 32MB, to match the limit that is set in RDK. This is necessary for
    // transmitting large pointclouds.
    builder.SetMaxSendMessageSize(viam::maximumGRPCByteLimit);
    builder.RegisterService(&slamService);

    // Start the SLAM gRPC server
    std::unique_ptr<grpc::Server> server(builder.BuildAndStart());

    // This log line is needed by rdk to get the port.
    LOG(INFO) << "Server listening on " << *selected_port << "\n";

    try {
        slamService.RunSLAM();
    } catch (std::exception& e) {
        LOG(ERROR)
            << "Stopping Cartographer: an error occurred during the run: "
            << e.what();
        std::terminate();
    }

    LOG(INFO) << "System shutdown";
}
