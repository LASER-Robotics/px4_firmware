#include <px4_platform_common/log.h>
#include <px4_platform_common/app.h>
#include <px4_platform_common/posix.h>
#include <fdeep/fdeep.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_angular_acceleration.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/actuator_outputs.h>
#include <Laser_NN.h>

Laser_NN::Laser_NN(){

}

Laser_NN::PrintExample(){
    PX4_INFO("Show!");
}

