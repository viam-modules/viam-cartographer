// This is an experimental integration of cartographer into RDK.

#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server_builder.h>

#include <iostream>
#include <thread>

#include "slam_service/slam_service.h"

int main(int argc, char** argv) {
    viam::SLAMServiceImpl slamService;
    slamService.Init(argc, argv);
    slamService.Start();

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
