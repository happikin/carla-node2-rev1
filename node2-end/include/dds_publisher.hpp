#include <dds_node.hpp>

class dds_publisher {
	private:
        std::mutex m_mutex;
		DDS::Publisher_var m_publisher;
		// narrow_writer_t m_message_writer;
		// DDS::DataWriter_var m_writer;
        std::map<std::string,DDS::DataWriter_ptr> m_writers;
	public:
		dds_publisher() = delete;
		dds_publisher(dds_node& _dds_node) {
			m_publisher = _dds_node.get_participant()->create_publisher(
                PUBLISHER_QOS_DEFAULT,
                0, OpenDDS::DCPS::DEFAULT_STATUS_MASK);

            if (!m_publisher) {
                std::cout << "create_publisher() failed\n";
            }
		}
        template <typename writer_t>
		int create_writer(
			dds_node& _dds_node,
            const std::string& _topic_name
		) {
            DDS::DataWriter_ptr m_writer = m_publisher->create_datawriter(
                _dds_node.get_topic(_topic_name),
                DATAWRITER_QOS_DEFAULT,
                0, OpenDDS::DCPS::DEFAULT_STATUS_MASK);

            if (!m_writer) {
                std::cout << "create_datawriter failed\n";
                return -1;
            }

            DDS::DataWriter_ptr m_message_writer = writer_t::_narrow(m_writer);

            if (!m_message_writer) {
                std::cout << "narrow() failed\n";
                return -1;
            }

            m_writers.insert(std::make_pair(_topic_name,m_message_writer));
        }
        int wait_for_subscriber(const std::string _topic_name) {
            DDS::StatusCondition_var condition = m_writers[_topic_name]->get_statuscondition();
            condition->set_enabled_statuses(DDS::PUBLICATION_MATCHED_STATUS);

            DDS::WaitSet_var ws = new DDS::WaitSet;
            ws->attach_condition(condition);

            ACE_DEBUG((LM_DEBUG,
                        ACE_TEXT("Block until subscriber is available\n")));

            while (true) {
                DDS::PublicationMatchedStatus matches;
                if (m_writers[_topic_name]->get_publication_matched_status(matches) != ::DDS::RETCODE_OK) {
                    ACE_ERROR_RETURN((LM_ERROR,
                                        ACE_TEXT("ERROR: %N:%l: main() -")
                                        ACE_TEXT(" get_publication_matched_status failed!\n")),
                                    1);
                }

                if (matches.current_count >= 1) break;

                DDS::ConditionSeq conditions;
                DDS::Duration_t timeout = { 60, 0 };
                if (ws->wait(conditions, timeout) != DDS::RETCODE_OK) {
                ACE_ERROR_RETURN((LM_ERROR,
                                    ACE_TEXT("ERROR: %N:%l: main() -")
                                    ACE_TEXT(" wait failed!\n")),
                                1);
                }
            }

            ACE_DEBUG((LM_DEBUG,
                        ACE_TEXT("Subscriber is available\n")));

            ws->detach_condition(condition);
        }
		template <typename payload_type, typename writer_t>
        void write(payload_type _payload, const std::string _topic_name) {
            {
                // std::lock_guard<std::mutex> lock_guard(m_mutex);
                DDS::ReturnCode_t error = writer_t::_narrow(m_writers[_topic_name])->write(_payload, DDS::HANDLE_NIL);

                if (error != DDS::RETCODE_OK) {
                    ACE_ERROR((LM_ERROR,
                            ACE_TEXT("ERROR: %N:%l: main() -")
                            ACE_TEXT(" write returned %d!\n"), error));
                }
            }
        }
		int wait_for_acknowledgments(const std::string _topic_name, DDS::Duration_t timeout = { 5, 0 }) {
			// DDS::Duration_t timeout = { 30, 0 };
			if (m_writers[_topic_name]->wait_for_acknowledgments(timeout) != DDS::RETCODE_OK) {
				ACE_ERROR_RETURN((LM_ERROR,
								ACE_TEXT("ERROR: %N:%l: main() -")
								ACE_TEXT(" wait_for_acknowledgments failed!\n")),
								1);
			}
		}
        ~dds_publisher() {
            this->m_publisher->delete_contained_entities();
        }
};