#include <ace/Log_Msg.h>
#include <ace/OS_NS_stdlib.h>

#include "RadarSensorListener.h"
#include "CarlaDataTypeSupportC.h"
#include "CarlaDataTypeSupportImpl.h"
#include "adas_features.hpp"

#include <iostream>

void
RadarSensorListener::on_requested_deadline_missed(
  DDS::DataReader_ptr /*reader*/,
  const DDS::RequestedDeadlineMissedStatus& /*status*/)
{
}

void
RadarSensorListener::on_requested_incompatible_qos(
  DDS::DataReader_ptr /*reader*/,
  const DDS::RequestedIncompatibleQosStatus& /*status*/)
{
}

void
RadarSensorListener::on_sample_rejected(
  DDS::DataReader_ptr /*reader*/,
  const DDS::SampleRejectedStatus& /*status*/)
{
}

void
RadarSensorListener::on_liveliness_changed(
  DDS::DataReader_ptr /*reader*/,
  const DDS::LivelinessChangedStatus& /*status*/)
{
}

void
RadarSensorListener::on_data_available(DDS::DataReader_ptr reader)
{
  CarlaData::RadarSensorDataReader_var reader_i =
    CarlaData::RadarSensorDataReader::_narrow(reader);

  if (!reader_i) {
    ACE_ERROR((LM_ERROR,
               ACE_TEXT("ERROR: %N:%l: on_data_available() -")
               ACE_TEXT(" _narrow failed!\n")));
    ACE_OS::exit(1);
  }

  CarlaData::RadarSensor message;
  DDS::SampleInfo info;

  const DDS::ReturnCode_t error = reader_i->take_next_sample(message, info);

  if (error == DDS::RETCODE_OK) {
    // std::cout << "SampleInfo.sample_rank = " << info.sample_rank << std::endl;
    // std::cout << "SampleInfo.instance_state = " << OpenDDS::DCPS::InstanceState::instance_state_mask_string(info.instance_state) << std::endl;

    if (info.valid_data) {
      std::cout 
        << std::to_string(std::chrono::system_clock::now().time_since_epoch().count())
        << " received CarlaData::RadarSensor["
        << "d:" << message.depth << ","
        << "v:" << message.velocity << ","
        << "az:" << message.azimuth << ","
        << "al:" << message.altitude << ","
        << "egov:" << message.ego_velocity << "]\n";

        /**
         * use Shrikant(ACC) here
        */

        {
          std::lock_guard<std::mutex> lock_guard(m_odometry_mutex);
          m_odometry_data = adas_features::run_acc_algo(message);
        }
    }

  } else {
    ACE_ERROR((LM_ERROR,
               ACE_TEXT("ERROR: %N:%l: on_data_available() -")
               ACE_TEXT(" take_next_sample failed!\n")));
  }
}

void
RadarSensorListener::on_subscription_matched(
  DDS::DataReader_ptr /*reader*/,
  const DDS::SubscriptionMatchedStatus& /*status*/)
{
}

void
RadarSensorListener::on_sample_lost(
  DDS::DataReader_ptr /*reader*/,
  const DDS::SampleLostStatus& /*status*/)
{
}

CarlaData::VehicleOdometry&
RadarSensorListener::get_odometry() {
  std::lock_guard<std::mutex> lock_guard(m_odometry_mutex);
  return m_odometry_data;
}