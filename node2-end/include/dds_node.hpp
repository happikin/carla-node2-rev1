#pragma once

#include <ace/Log_Msg.h>

#include <dds/DdsDcpsInfrastructureC.h>
#include <dds/DdsDcpsSubscriptionC.h>
#include <dds/DdsDcpsPublicationC.h>

#include <dds/DCPS/Marked_Default_Qos.h>
#include <dds/DCPS/Service_Participant.h>
#include <dds/DCPS/WaitSet.h>

#include <dds/DCPS/StaticIncludes.h>
#ifdef ACE_AS_STATIC_LIBS
#  include <dds/DCPS/RTPS/RtpsDiscovery.h>
#  include <dds/DCPS/transport/rtps_udp/RtpsUdp.h>
#endif

#include <mutex>
#include <map>

class dds_node {
	private:
		DDS::DomainParticipantFactory_var dpf;
    	DDS::DomainParticipant_var participant;
		std::map<std::string,DDS::Topic_ptr> m_topics;
		std::mutex m_mutex;

	public:
		dds_node(int argc, char **argv) {
			{
				std::lock_guard<std::mutex> lock(m_mutex);
				dpf = TheParticipantFactoryWithArgs(argc, argv);
				participant = dpf->create_participant(
					42,	PARTICIPANT_QOS_DEFAULT,
					0,	OpenDDS::DCPS::DEFAULT_STATUS_MASK);
				if (!participant) {
					std::cerr << "create_participant() failed\n";
				}
			}
		}
		// template <typename type_support_impl>
		// void register_type() {
		// 	{
		// 		std::lock_guard<std::mutex> lock(m_mutex);
		// 	}
		// }
		template <typename type_support, typename type_support_impl>
		void create_topic(const std::string _topic_name) {
			{
				std::lock_guard<std::mutex> lock(m_mutex);
				type_support ts = new type_support_impl;
				auto err = ts->register_type(participant,"");
				if(err != DDS::RETCODE_OK) {
					std::cerr << "register_type() failed\n";
				}
				CORBA::String_var type_name = ts->get_type_name();
				DDS::Topic_ptr topic = participant->create_topic(
					_topic_name.c_str(), type_name,
					TOPIC_QOS_DEFAULT, 0, OpenDDS::DCPS::DEFAULT_STATUS_MASK);
				if(!topic) {
					std::cout << "create_topic() failed\n";
				} else m_topics.insert(std::make_pair(_topic_name,topic));
				ts = nullptr;
			}
		}
		DDS::DomainParticipant_ptr get_participant() {
			{
				std::lock_guard<std::mutex> lock(m_mutex);
				return this->participant;
			}
		}
		DDS::Topic_ptr get_topic(const std::string _topic_name) {
			{
				std::lock_guard<std::mutex> lock(m_mutex);
				return this->m_topics[_topic_name];
			}
		}
		~dds_node() {
			participant->delete_contained_entities();
			dpf->delete_participant(participant);
			TheServiceParticipant->shutdown();
		}
};