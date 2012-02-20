#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include "fake_sync_template/fake-synchronizer-impl.h"
using namespace sensor_msgs;
using namespace message_filters;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_sync");
  typedef FakeSynchronizer<Image, JointState> SyncNode;
  SyncNode sync_node;
  return 0;
}
