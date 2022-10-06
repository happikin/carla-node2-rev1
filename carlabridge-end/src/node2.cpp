#include <dds_subscriber.hpp>
#include <dds_publisher.hpp>
#include "VehicleOdometryListener.h"
#include "RadarSensorListener.h"
#include "ImageSensorListener.hpp"
#include "CarlaDataTypeSupportImpl.h"
#include "adas_features.hpp"
#include <topics.hpp>
#include <thread>

std::atomic<uint8_t> close_flag{0};
std::mutex odometry_mutex;
CarlaData::VehicleOdometry g_odometry;

void signal_handler(int _signal) {
	close_flag.store(1);
	std::cout << ": ran signal_handler()\n";
}

int main(int argc, char *argv[]) {
	signal(SIGINT,signal_handler);
	dds_node this_node(argc,argv);
	this_node.create_topic<
		CarlaData::RadarSensorTypeSupport_ptr,
		CarlaData::RadarSensorTypeSupportImpl
	> (topic_names[0].c_str());
	this_node.create_topic<
		CarlaData::VehicleOdometryTypeSupport_ptr,
		CarlaData::VehicleOdometryTypeSupportImpl
	> (topic_names[1].c_str());
	this_node.create_topic<
		CarlaData::ImageSensorTypeSupport_ptr,
		CarlaData::ImageSensorTypeSupportImpl
	> (topic_names[2].c_str());

	dds_subscriber subscriber(this_node);
	dds_publisher publisher(this_node);

	RadarSensorListener *radar_listener = new RadarSensorListener();
	ImageSensorListener *image_listener = new ImageSensorListener();
	
	subscriber.create_reader<CarlaData::RadarSensorDataReader>(
		this_node,topic_names[0],(radar_listener));
	subscriber.create_reader<CarlaData::ImageSensorDataReader>(
		this_node,topic_names[2],(image_listener));
	publisher.create_writer<CarlaData::VehicleOdometryDataWriter>(
		this_node, topic_names[1]);

	std::thread sub_thread_radar([&](){
		subscriber.wait_for_publisher(topic_names[0]);
	});
	std::thread sub_thread_image([&](){
		subscriber.wait_for_publisher(topic_names[2]);
	});

	/**
	 * making this thread in detached state might cause
	 * itself to be pre-empted too much due to low priority
	 * thus never gets a subscriber
	*/

	std::thread([&](){
		publisher.wait_for_subscriber(topic_names[1]);
		while(close_flag.load() == 0) {
			publisher.write<CarlaData::VehicleOdometry,CarlaData::VehicleOdometryDataWriter>(
				radar_listener->get_odometry(), topic_names[1]);
			std::this_thread::sleep_for(
				std::chrono::microseconds(100)
			);
		}
		std::cout << "waiting for acknowledgments on topic[" << topic_names[1] << "]\n";
		publisher.wait_for_acknowledgments(topic_names[1]);
	}).detach();

	std::thread([&](){
		while(close_flag.load() == 0) {
			// cv::imshow("test1",image_listener->get_image());
			// cv::waitKey(1000);
			cv::Mat img = image_listener->get_image();
			if(img.size().height > 0 && img.size().width > 0) {
				adas_features::run_object_detection_algo(img);
			}
			std::this_thread::sleep_for(
				std::chrono::microseconds(100)
			);
		}
	}).detach();

	while(close_flag.load() == 0);
	sub_thread_radar.detach();
	sub_thread_image.detach();
	// sub_thread.join();
    return 0;
}
