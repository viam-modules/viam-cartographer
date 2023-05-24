#include "slam_service_c.h"
#include "../slam_service/slam_service.h"

int TestCgoC(int num) {
    return static_cast<int>(viam::SensorId::SensorType::IMU) + num;
}
