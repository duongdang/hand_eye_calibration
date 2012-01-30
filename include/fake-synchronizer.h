#ifndef _fake_synchronizer_h
#define _fake_synchronizer_h

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <ros/console.h>

template <typename T1, typename T2>
class FakeSynchronizer
{
public:
  explicit FakeSynchronizer();
  virtual ~FakeSynchronizer(){};
  void callback(const boost::shared_ptr<T1 const>& m1, const boost::shared_ptr<T2 const>& m2);
  ros::Publisher pub;
};

#endif
