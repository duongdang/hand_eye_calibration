#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <fstream>
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <ros/console.h>
#include <visp/vpCalibration.h>
#include <ros/console.h>
#include <fstream>
#include "yaml-cpp/yaml.h"


using namespace sensor_msgs;
using namespace message_filters;

namespace {
  YAML::Emitter& operator << (YAML::Emitter& out, const vpHomogeneousMatrix& m)
  {
    out << YAML::Flow;
    out << YAML::BeginSeq;
    for (unsigned i = 0; i < 4; i++)
      for (unsigned j = 0; j < 4; j++)
      {
        out << m[i][j];
      }

    out << YAML::EndSeq;
    return out;
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "calibrator", ros::init_options::AnonymousName);

  std::ifstream fin( argv[1]);
  YAML::Parser parser(fin);
  YAML::Node doc;
  parser.GetNextDocument(doc);
  vpHomogeneousMatrix  	cMo[doc.size()];
  vpHomogeneousMatrix  	rMe[doc.size()];
  unsigned nbPose(doc.size());
  for(unsigned i = 0; i < doc.size(); i++){
    std::auto_ptr<YAML::Node> sample = doc[i].Clone();

    for(YAML::Iterator iit = sample->begin(); iit != sample->end(); ++iit) {
      std::string key;
      iit.first() >> key;
      std::auto_ptr<YAML::Node> mat_node = iit.second().Clone();
      double d[mat_node->size()];
      for (unsigned j = 0; j < mat_node->size(); j++)
        {
          (*mat_node)[j] >> d[j];
        }
      if (key == "T_chessboard")
        {
          cMo[i] << d;
        }
      else if (key == "T_link")
        {
          rMe[i] << d;
        }
      else
        ROS_FATAL_STREAM("Invalid matrix " << key );
    }
  }
  vpHomogeneousMatrix eMc;
  vpCalibration::calibrationTsai(nbPose, cMo, rMe, eMc);
  YAML::Emitter yout;
  yout << eMc;
  std::cout << eMc << std::endl;
  std::ofstream outf(argv[2], std::ios::out);
  outf << yout.c_str() << std::endl;
  outf.close();
  return 0;
}
