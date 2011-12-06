#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <ros/console.h>
using namespace sensor_msgs;
using namespace message_filters;

class FakeSynchronizer
{
public:
  explicit FakeSynchronizer();
  virtual ~FakeSynchronizer(){};
  void callback(const ImageConstPtr& image, const JointState::ConstPtr& state);
  ros::Publisher pub;
};

void  FakeSynchronizer::callback(const ImageConstPtr& image, const JointState::ConstPtr& state)
{
    float diff = float(int(image->header.stamp.nsec) - int(state->header.stamp.nsec))/1e9 +
      float(image->header.stamp.sec) - float(state->header.stamp.sec);

    // ROS_INFO_STREAM(image->header.stamp << " "
    //                 << state->header.stamp << " "
    //                 << image->header.stamp - state->header.stamp <<" "<< diff);
    JointState synced_state(*state);
    synced_state.header.stamp = image->header.stamp;
    pub.publish(synced_state);

  }

FakeSynchronizer::FakeSynchronizer()
{
    ros::NodeHandle nh("~");
    std::string in_topic, aligned_topic, out_topic;

    nh.getParam("in", in_topic);
    nh.getParam("aligned", aligned_topic);
    nh.getParam("out", out_topic);

    message_filters::Subscriber<JointState> in_sub(nh, in_topic, 1);
    message_filters::Subscriber<Image> aligned_sub(nh, aligned_topic, 1);

    typedef sync_policies::ApproximateTime<Image, JointState> MySyncPolicy;

    Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), aligned_sub, in_sub);
    sync.registerCallback(boost::bind(&FakeSynchronizer::callback, this , _1, _2));
    pub = nh.advertise<JointState>(out_topic, 5);
    ros::spin();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_sync");

  FakeSynchronizer sync;
  return 0;
}
