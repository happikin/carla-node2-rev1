#include <dds_publisher.hpp>
#include <dds_subscriber.hpp>
#include "VehicleOdometryListener.h"
#include "RadarSensorListener.h"
#include "CarlaDataTypeSupportImpl.h"
#include <topics.hpp>
#include <atomic>
#include <thread>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/highgui.hpp>

std::atomic<int> close_flag{0};

void signal_handler(int _signal) {
	close_flag.store(1);
	std::cout << ": ran signal_handler()\n";
}

int main(int argc, char *argv[]) {

	char *args[] = {"./node1","-DCPSConfigFile","../rtps.ini"};
	signal(SIGINT,signal_handler);
	dds_node this_node(3,args);

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

	dds_publisher publisher(this_node);
	dds_subscriber subscriber(this_node);

	publisher.create_writer<CarlaData::RadarSensorDataWriter>(
		this_node, topic_names[0]);
	publisher.create_writer<CarlaData::ImageSensorDataWriter>(
		this_node, topic_names[2]);
    subscriber.create_reader<CarlaData::VehicleOdometryDataReader>(
		this_node,topic_names[1],(new VehicleOdometryListener));

	std::thread([&](){
		publisher.wait_for_subscriber(topic_names[0]);
		for(size_t i{}; i<100; i++) {
			CarlaData::RadarSensor m;
			m.depth = i;
			m.velocity = i+2;
			m.azimuth = i*2;
			m.altitude = i*3;
			publisher.write<CarlaData::RadarSensor,CarlaData::RadarSensorDataWriter>(
				m, topic_names[0]);
			std::this_thread::sleep_for(
				std::chrono::milliseconds(250)
			);
		}
		publisher.wait_for_acknowledgments(topic_names[0]);
	}).detach();
	
	std::thread([&](){
		cv::Mat read_image = cv::imread("/home/fev/Pictures/vibe1.png");
		publisher.wait_for_subscriber(topic_names[2]);
		while(close_flag.load() == 0) {

			CarlaData::ImageSensor image;
			long image_size = read_image.rows * read_image.cols * read_image.elemSize();

			image.height = read_image.rows;
			image.width = read_image.cols;
			image.pixel_size = read_image.elemSize();
			image.image_type = read_image.type();
			image.raw_data.replace(image_size, image_size, read_image.data, false);
			// cv::waitKey(1000);

			publisher.write<CarlaData::ImageSensor,CarlaData::ImageSensorDataWriter>(
				image, topic_names[2]);
			std::this_thread::sleep_for(
				std::chrono::milliseconds(250)
			);

			// exit(1);
		}
		publisher.wait_for_acknowledgments(topic_names[2]);
	}).detach();
	
	std::thread sub_thread([&](){
		subscriber.wait_for_publisher(topic_names[1]);
	});

	while(close_flag.load() == 0);
	sub_thread.detach();
	// sub_thread.join();
    return 0;
}
