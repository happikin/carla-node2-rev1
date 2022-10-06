progress:
 - generic api for creating publisher and subscriber with one participant is ready
 - two applications were created using this api: dds_subscriber.cpp & dds_publisher.cpp
 - one was publishing on two different topics and other was subscribing to the two topics
 NOTE: there was only one publisher but had two data writers same was the case with subscriber
 - Date[Sep-20-22]: was able to establish connection of type (a)
goals:
 (a) make a publisher and subscriber with one participant in one node and same with other
 test if data is getting exhanged

progress:
 - this project version has the same node1 and node2 cpp files
 but with proper CarlaData.idl to sipport RadarSensor data
 and VehicleOdometry data
 - node1 code has been integrated into the CarlaClient::dds_node_proc() thread in carlabride
 and has been tested with node1 running on other terminal: data is still getting exchanged

NOTE:
 `DDS::DataReaderQos reader_qos;
			m_subscriber->get_default_datareader_qos(reader_qos);
			reader_qos.reliability.kind = DDS::BEST_EFFORT_RELIABILITY_QOS;`
these lines in dds_subscriber.hpp can be changed to tingle wth the speed of data transmission

 - in node1 publisher variable is being used by two threads, so maybe we need lock on it