/*
 *
 *
 * Distributed under the OpenDDS License.
 * See: http://www.opendds.org/license.html
 */

#include <ace/Log_Msg.h>
#include <ace/OS_NS_stdlib.h>

#include "VehicleOdometryListener.h"
#include "CarlaDataTypeSupportC.h"
#include "CarlaDataTypeSupportImpl.h"

#include <iostream>

void
VehicleOdometryListener::on_requested_deadline_missed(
  DDS::DataReader_ptr /*reader*/,
  const DDS::RequestedDeadlineMissedStatus& /*status*/)
{
}

void
VehicleOdometryListener::on_requested_incompatible_qos(
  DDS::DataReader_ptr /*reader*/,
  const DDS::RequestedIncompatibleQosStatus& /*status*/)
{
}

void
VehicleOdometryListener::on_sample_rejected(
  DDS::DataReader_ptr /*reader*/,
  const DDS::SampleRejectedStatus& /*status*/)
{
}

void
VehicleOdometryListener::on_liveliness_changed(
  DDS::DataReader_ptr /*reader*/,
  const DDS::LivelinessChangedStatus& /*status*/)
{
}

void
VehicleOdometryListener::on_data_available(DDS::DataReader_ptr reader)
{
  CarlaData::VehicleOdometryDataReader_var reader_i =
    CarlaData::VehicleOdometryDataReader::_narrow(reader);

  if (!reader_i) {
    ACE_ERROR((LM_ERROR,
               ACE_TEXT("ERROR: %N:%l: on_data_available() -")
               ACE_TEXT(" _narrow failed!\n")));
    ACE_OS::exit(1);
  }

  CarlaData::VehicleOdometry message;
  DDS::SampleInfo info;

  const DDS::ReturnCode_t error = reader_i->take_next_sample(message, info);

  if (error == DDS::RETCODE_OK) {
    // std::cout << "SampleInfo.sample_rank = " << info.sample_rank << std::endl;
    // std::cout << "SampleInfo.instance_state = " << OpenDDS::DCPS::InstanceState::instance_state_mask_string(info.instance_state) << std::endl;

    if (info.valid_data) {
      std::cout
        << std::to_string(std::chrono::system_clock::now().time_since_epoch().count())
        << " received CarlaData::VehicleOdometry["
        << "t:" << message.throttle << ","
        << "b:" << message.brake << ","
        << "s:" << message.steering
        << "]\n";
      {
        std::lock_guard<std::mutex> lock_guard(m_odometry_mutex);
        m_odometry_data.throttle = message.throttle;
        m_odometry_data.brake = message.brake;
        m_odometry_data.steering = message.steering;
      }
    }

  } else {
    ACE_ERROR((LM_ERROR,
               ACE_TEXT("ERROR: %N:%l: on_data_available() -")
               ACE_TEXT(" take_next_sample failed!\n")));
  }
}

void
VehicleOdometryListener::on_subscription_matched(
  DDS::DataReader_ptr /*reader*/,
  const DDS::SubscriptionMatchedStatus& /*status*/)
{
}

void
VehicleOdometryListener::on_sample_lost(
  DDS::DataReader_ptr /*reader*/,
  const DDS::SampleLostStatus& /*status*/)
{
}

CarlaData::VehicleOdometry
VehicleOdometryListener::get_odometry() {
  std::lock_guard<std::mutex> lock_guard(m_odometry_mutex);
  return m_odometry_data;
}