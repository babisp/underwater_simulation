#ifndef TESTSENSOR_ECHO_H_
#define TESTSENSOR_ECHO_H_
#include "SimulatedDevice.h"
#include <ros/ros.h>
#include "ConfigXMLParser.h"
#include "ROSInterface.h"
#include <std_msgs/Int32.h>
using namespace uwsim;

class TestSensor : public SimulatedDevice
{
public:
  int frequency;

  TestSensor(TestSensor_Config * cfg);
};

class TestSensor_Config : public SimulatedDeviceConfig
{
public:
  //XML members
  int frequency;
  //constructor
  TestSensor_Config(std::string type_) :
  SimulatedDeviceConfig(type_)
  {
  }
};

class TestSensor_Factory : public SimulatedDeviceFactory
{
public:
  //this is the only place the device/interface type is set
  TestSensor_Factory(std::string type_ = "TestSensor") :
  SimulatedDeviceFactory(type_)
  {
  };

  SimulatedDeviceConfig::Ptr processConfig(const xmlpp::Node* node, ConfigFile * config);
  bool applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *sceneBuilder, size_t iteration);
  std::vector<boost::shared_ptr<ROSInterface> > getInterface(ROSInterfaceInfo & rosInterface,
    std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile);
};

class TestSensor_ROSPublisher : public ROSPublisherInterface
{
  TestSensor * dev;
public:
  TestSensor_ROSPublisher(TestSensor *dev, std::string topic, int rate) :
      ROSPublisherInterface(topic, rate), dev(dev)
  {
  }

  void createPublisher(ros::NodeHandle &nh);
  void publish();

  ~TestSensor_ROSPublisher()
  {
  }
};

#endif
