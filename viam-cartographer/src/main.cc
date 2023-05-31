// This is an experimental integration of cartographer into RDK.

#include <iostream>
#include <thread>

#include "slam_service/slam_service.h"



#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server_builder.h>

int main(int argc, char** argv) {
    viam::SLAMServiceImpl slamService;
    slamService.Init(argc, argv);
    std::unique_ptr<grpc::Server> server = slamService.Start();

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
