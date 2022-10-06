#ifndef CARLA_DDS_BRIDGE_H
#define CARLA_DDS_BRIDGE_H

#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <fstream>
#include <math.h>
#include <mutex>
#include <cmath>
#include <signal.h>

#include "carla/client/Client.h"
#include "carla/client/ActorList.h"
#include "carla/client/BlueprintLibrary.h"
#include "carla/sensor/data/ImageTmpl.h"
#include "carla/sensor/data/RadarMeasurement.h"
#include "carla/client/ServerSideSensor.h"
#include "carla/client/Vehicle.h"
#include "carla/client/DebugHelper.h"
#include "dds_node.hpp"
#include "dds_publisher.hpp"
#include "dds_subscriber.hpp"
#include "VehicleOdometryListener.h"
#include "RadarSensorListener.h"
#include "CarlaDataTypeSupportImpl.h"
#include "topics.hpp"

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>

uint8_t terminateFlag=0;

class CarlaClient {
	private:
		const char* IP;
		const int port;
		int argc;
		char **argv;
		carla::client::Client carlaClient;
		carla::client::DebugHelper dh;
	
		carla::SharedPtr<carla::client::Map> currentMap;
		carla::SharedPtr<carla::client::Actor> carlaCarActor;
		carla::SharedPtr<carla::client::Vehicle> otherCarActor;
		carla::SharedPtr<carla::client::Vehicle> carlaCarActor_downcast;
		//carla::SharedPtr<carla::client::ClientSideSensor> carlaCamActor;
		//carla::SharedPtr<carla::client::ClientSideSensor> carlaRadarActor;
		carla::SharedPtr<carla::client::ServerSideSensor> carlaCamActor;
		carla::SharedPtr<carla::client::ServerSideSensor> carlaRadarActor;
		carla::SharedPtr<carla::client::Actor> spectator;
	
		carla::geom::Transform carTransform;
		carla::geom::Transform spectator_transform;
		carla::geom::Location spectator_location;
	
		std::atomic<float> steeringSetpoint;
		std::atomic<float> accelerationSetpoint;
		std::atomic<float> velocitySetpoint;
		std::atomic<double> latestFrame;
		std::atomic<int> close_flag{0};

		uint8_t controllerThreadFlag;
		uint8_t imgUpdateFlag;
		uint8_t radarUpdateFlag;
		uint8_t stopListening_flag;
		uint16_t currentImageFrameVal;
		float error;
		float prev_error;
		float sum_error;

		std::chrono::time_point<std::chrono::system_clock> prev_time;

		CarlaData::RadarSensor m_radar_data;
		std::mutex m_radar_mutex;
		
		CarlaData::VehicleOdometry m_odometry_data;
		std::mutex m_odometry_mutex;

		cv::Mat m_image_data;
		std::mutex m_image_mutex;

	public:
		std::chrono::time_point<std::chrono::system_clock> simulationStartTime;
		void StopListening();
		void pidController();
		void radar_callback(carla::SharedPtr<carla::sensor::SensorData> rawSensorDataPtr);
		void ImgCbk(carla::SharedPtr<carla::sensor::SensorData> rawSensorDataPtr);
		void updateCarCamera();
		void generateWorld();
		void dds_node_proc();
		void vehicle_control_proc();
		void signal_handler(int);
		CarlaClient(
			const char* clientIP,
			int clientPort,
			int argc,
			char **argv
		);
		~CarlaClient();

		// void updateCarlaClient(carla::SharedPtr<carla::sensor::data::RadarMeasurement> RadarCloudPtr);
		// void updateCarlaSensorData_Radar(carla::SharedPtr<carla::sensor::data::RadarMeasurement> RadarCloudPtr);
		// void updateCarlaSensorData_Img(carla::SharedPtr<carla::sensor::data::ImageTmpl<carla::sensor::data::Color>> ImagePtr);
		// void updateCarlaSensorData_Odometry();
		// void updateCarlaSensorData();
		// void convertCarlaImgToRawImg(carla::SharedPtr<carla::sensor::data::ImageTmpl<carla::sensor::data::Color>> ImagePtr);
		// void ddsTransmitFn();

		//void updateCarlaClient(ddsCarlaSensorData* ddsRcvDataPtr);
		// void updateCarlaClient(float acceleration, float steering,double frameNumber,float velocity);
		// void set_carlaDdsBridgeCbk(std::function<void (ddsCarlaSensorData*)> cbkPtr);
		// void initializeCarlaSensorData();
};
#endif