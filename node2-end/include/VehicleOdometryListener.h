#ifndef VEHICLEODOMETRY_LISTENER_H
#define VEHICLEODOMETRY_LISTENER_H

#include <ace/Global_Macros.h>

#include <dds/DdsDcpsSubscriptionC.h>
#include <dds/DCPS/LocalObject.h>
#include <dds/DCPS/Definitions.h>
#include "CarlaDataTypeSupportImpl.h"
#include <chrono>
#include <mutex>

class VehicleOdometryListener
  : public virtual OpenDDS::DCPS::LocalObject<DDS::DataReaderListener> {
private:
  CarlaData::VehicleOdometry m_odometry_data;
  std::mutex m_odometry_mutex;
public:
  virtual void on_requested_deadline_missed(
    DDS::DataReader_ptr reader,
    const DDS::RequestedDeadlineMissedStatus& status);

  virtual void on_requested_incompatible_qos(
    DDS::DataReader_ptr reader,
    const DDS::RequestedIncompatibleQosStatus& status);

  virtual void on_sample_rejected(
    DDS::DataReader_ptr reader,
    const DDS::SampleRejectedStatus& status);

  virtual void on_liveliness_changed(
    DDS::DataReader_ptr reader,
    const DDS::LivelinessChangedStatus& status);

  virtual void on_data_available(
    DDS::DataReader_ptr reader);

  virtual void on_subscription_matched(
    DDS::DataReader_ptr reader,
    const DDS::SubscriptionMatchedStatus& status);

  virtual void on_sample_lost(
    DDS::DataReader_ptr reader,
    const DDS::SampleLostStatus& status);

  CarlaData::VehicleOdometry get_odometry();
};

#endif /* DATAREADER_LISTENER_IMPL_H */
