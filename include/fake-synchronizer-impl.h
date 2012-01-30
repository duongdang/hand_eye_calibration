#include "fake-synchronizer.h"
using namespace sensor_msgs;

#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
template <typename T1, typename T2>
FakeSynchronizer<T1, T2>::FakeSynchronizer()
{
    ros::NodeHandle nh("~");
    std::string topic1, topic2, topic_out;

    nh.getParam("topic1", topic1);
    nh.getParam("topic2", topic2);
    nh.getParam("out", topic_out);

    message_filters::Subscriber<T1> sub1(nh, topic1, 1);
    message_filters::Subscriber<T2> sub2(nh, topic2, 1);

    typedef message_filters::sync_policies::ApproximateTime<T1, T2> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), sub1, sub2);
    sync.registerCallback(boost::bind(&FakeSynchronizer<T1, T2>::callback, this , _1, _2));
    pub = nh.advertise<T2>(topic_out, 5);
    ros::spin();
}

template <typename T1, typename T2>
void  FakeSynchronizer<T1, T2>::callback(const boost::shared_ptr<T1 const>& m1,
                                         const boost::shared_ptr<T2 const>& m2)
{
  T2 out(*m2);
  out.header.stamp = m1->header.stamp;
  pub.publish(out);
}

