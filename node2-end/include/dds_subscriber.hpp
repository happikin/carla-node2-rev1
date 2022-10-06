#include <dds_node.hpp>


class dds_subscriber {
	private:
		DDS::Subscriber_var m_subscriber;
		// DDS::DataReaderListener_ptr m_listener;
		// narrow_reader_t m_reader_i;
		// DDS::DataReader_var m_reader;
		std::map<std::string,DDS::DataReader_ptr> m_readers;
	public:
		dds_subscriber() = delete;
		dds_subscriber(dds_node& _dds_node) {
			m_subscriber = _dds_node.get_participant()->create_subscriber(
									SUBSCRIBER_QOS_DEFAULT, 0,
									OpenDDS::DCPS::DEFAULT_STATUS_MASK
								);
			if (!m_subscriber) {
				std::cerr << "create subscriber failed\n";
			}
		}
		template <typename reader_t>
		int create_reader(
			dds_node& _dds_node,
			const std::string _topic_name,
			DDS::DataReaderListener_ptr _listener
		) {
			DDS::DataReaderQos reader_qos;
			m_subscriber->get_default_datareader_qos(reader_qos);
			// reader_qos.reliability.kind = DDS::RELIABLE_RELIABILITY_QOS;
			reader_qos.reliability.kind = DDS::BEST_EFFORT_RELIABILITY_QOS;
			DDS::DataReader_ptr m_reader =
				m_subscriber->create_datareader(_dds_node.get_topic(_topic_name),
											reader_qos,
											_listener,
											OpenDDS::DCPS::DEFAULT_STATUS_MASK);

			if (!m_reader) {
				ACE_ERROR_RETURN((LM_ERROR,
								ACE_TEXT("ERROR: %N:%l: main() -")
								ACE_TEXT(" create_datareader failed!\n")),
								1);
			}

			DDS::DataReader_ptr temp_ptr = reader_t::_narrow(m_reader);
			if (!temp_ptr) {
				ACE_ERROR_RETURN((LM_ERROR,
								ACE_TEXT("ERROR: %N:%l: main() -")
								ACE_TEXT(" _narrow failed!\n")),
								1);
			}
			m_readers.insert(std::make_pair(_topic_name,temp_ptr));
		}

		int wait_for_publisher(const std::string _topic_name) {
			DDS::StatusCondition_var condition = m_readers[_topic_name]->get_statuscondition();
			condition->set_enabled_statuses(DDS::SUBSCRIPTION_MATCHED_STATUS);

			DDS::WaitSet_var ws = new DDS::WaitSet;
			ws->attach_condition(condition);

			while (true) {
				DDS::SubscriptionMatchedStatus matches;
				if (m_readers[_topic_name]->get_subscription_matched_status(matches) != DDS::RETCODE_OK) {
					ACE_ERROR_RETURN((LM_ERROR,
										ACE_TEXT("ERROR: %N:%l: main() -")
										ACE_TEXT(" get_subscription_matched_status failed!\n")),
									1);
				}

				if (matches.current_count == 0 && matches.total_count > 0) {
					break;
				}

				DDS::ConditionSeq conditions;
				DDS::Duration_t timeout = { 60, 0 };
				if (ws->wait(conditions, timeout) != DDS::RETCODE_OK) {
					ACE_ERROR_RETURN((LM_ERROR,
										ACE_TEXT("ERROR: %N:%l: main() -")
										ACE_TEXT(" wait failed!\n")),
									1);
				}
			}

			ws->detach_condition(condition);
		}
		~dds_subscriber() {
			for(auto& item:m_readers) {
				item.second->delete_contained_entities();
			}
			this->m_subscriber->delete_contained_entities();
		}
};