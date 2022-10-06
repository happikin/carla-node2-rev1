/*
 *
 *
 * Distributed under the OpenDDS License.
 * See: http://www.opendds.org/license.html
 */

#include <ace/Log_Msg.h>
#include <ace/OS_NS_stdlib.h>

#include "ImageSensorListener.hpp"
#include "CarlaDataTypeSupportC.h"
#include "CarlaDataTypeSupportImpl.h"

#include <iostream>

void
ImageSensorListener::on_requested_deadline_missed(
  DDS::DataReader_ptr /*reader*/,
  const DDS::RequestedDeadlineMissedStatus& /*status*/)
{
}

void
ImageSensorListener::on_requested_incompatible_qos(
  DDS::DataReader_ptr /*reader*/,
  const DDS::RequestedIncompatibleQosStatus& /*status*/)
{
}

void
ImageSensorListener::on_sample_rejected(
  DDS::DataReader_ptr /*reader*/,
  const DDS::SampleRejectedStatus& /*status*/)
{
}

void
ImageSensorListener::on_liveliness_changed(
  DDS::DataReader_ptr /*reader*/,
  const DDS::LivelinessChangedStatus& /*status*/)
{
}

void
ImageSensorListener::on_data_available(DDS::DataReader_ptr reader)
{
  CarlaData::ImageSensorDataReader_var reader_i =
    CarlaData::ImageSensorDataReader::_narrow(reader);

  if (!reader_i) {
    ACE_ERROR((LM_ERROR,
               ACE_TEXT("ERROR: %N:%l: on_data_available() -")
               ACE_TEXT(" _narrow failed!\n")));
    ACE_OS::exit(1);
  }

  CarlaData::ImageSensor image;
  DDS::SampleInfo info;

  const DDS::ReturnCode_t error = reader_i->take_next_sample(image, info);

  if (error == DDS::RETCODE_OK) {
    // std::cout << "SampleInfo.sample_rank = " << info.sample_rank << std::endl;
    // std::cout << "SampleInfo.instance_state = " << OpenDDS::DCPS::InstanceState::instance_state_mask_string(info.instance_state) << std::endl;

    if (info.valid_data) {
      std::cout 
        << std::to_string(std::chrono::system_clock::now().time_since_epoch().count())
        << " received CarlaData::ImageSensor\n";

      {
        std::lock_guard<std::mutex> lock_guard(m_mutex);
        m_received_image = cv::Mat(
          image.height, image.width,
          image.image_type, image.raw_data.get_buffer());
      }
			// cv::imshow("test1",m_received_image);
			// cv::waitKey(500);
    }

  } else {
    ACE_ERROR((LM_ERROR,
               ACE_TEXT("ERROR: %N:%l: on_data_available() -")
               ACE_TEXT(" take_next_sample failed!\n")));
  }
}

void
ImageSensorListener::on_subscription_matched(
  DDS::DataReader_ptr /*reader*/,
  const DDS::SubscriptionMatchedStatus& /*status*/)
{
}

void
ImageSensorListener::on_sample_lost(
  DDS::DataReader_ptr /*reader*/,
  const DDS::SampleLostStatus& /*status*/)
{
}

cv::Mat&
ImageSensorListener::get_image() {
  {
    std::lock_guard<std::mutex> lock_guard(m_mutex);
    return m_received_image;
  }
}