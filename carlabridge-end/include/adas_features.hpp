#include "CarlaDataTypeSupportImpl.h"
#include "carla/geom/Vector3D.h"
#include <opencv4/opencv2/core.hpp>
class adas_features
{
public:
    static CarlaData::VehicleOdometry
    run_acc_algo(
        /* carla::geom::Vector3D &_velocity_vector,  = egoVehicleDowncast->GetVelocity(); */
        CarlaData::RadarSensor &_radar_data
    ) {

        double time_to_stop = 0;
        double default_distance = 2.0;
        double time_gap = 1.4;
        double ego_vehicle_velocity = 0.0;
        double safe_distance = 0;
        double relative_distance = 0;
        double cruise_velocity = 1.0;
        double accelaration = 0;
        double final_velocity = 0;
        double differnetial_time = 0;
        double lead_vehicle_velocity = 0;

        CarlaData::VehicleOdometry local_odometry;
        
        ego_vehicle_velocity = _radar_data.ego_velocity;
        relative_distance = _radar_data.depth;
        final_velocity = ego_vehicle_velocity + _radar_data.velocity;
        safe_distance = default_distance + time_gap * ego_vehicle_velocity;

        if (relative_distance >= safe_distance) {
            local_odometry.throttle = cruise_velocity;
            local_odometry.brake = 0.0;
            local_odometry.steering = 0.0;
        } else {
            time_to_stop = (2 * (safe_distance - default_distance)) / (ego_vehicle_velocity + final_velocity);
            accelaration = (final_velocity - ego_vehicle_velocity) / time_to_stop;
            differnetial_time += time_to_stop / 1000;
            local_odometry.brake = differnetial_time;
            local_odometry.throttle = 0.0;
            local_odometry.steering = 0.0;
        }

        return local_odometry;
    }

    static void run_object_detection_algo(
        cv::Mat _received_image
    ) {
        std::cout << "running object detection algo\n";
    }
};