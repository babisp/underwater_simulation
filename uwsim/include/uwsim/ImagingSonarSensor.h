#ifndef ImagingSonarSensor_ECHO_H_
#define ImagingSonarSensor_ECHO_H_
#include "SimulatedDevice.h"
#include <ros/ros.h>
#include "ConfigXMLParser.h"
#include "ROSInterface.h"
#include <std_msgs/Int32.h>
using namespace uwsim;

class ImagingSonarSensor_Config : public SimulatedDeviceConfig {
public:
  //XML members
  int frequency;
  //constructor
  ImagingSonarSensor_Config(std::string type_) :
    SimulatedDeviceConfig(type_) {
  }
};

class ImagingSonarSensor_Factory : public SimulatedDeviceFactory {
public:
  //this is the only place the device/interface type is set
  ImagingSonarSensor_Factory(std::string type_ = "ImagingSonarSensor") :
    SimulatedDeviceFactory(type_) {
  };

  SimulatedDeviceConfig::Ptr processConfig(const xmlpp::Node* node, ConfigFile * config);
  bool applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *sceneBuilder, size_t iteration);
  std::vector<boost::shared_ptr<ROSInterface> > getInterface(ROSInterfaceInfo & rosInterface,
      std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile);
};

class ImagingSonarSensor : public SimulatedDevice {
public:
  int frequency;

  ImagingSonarSensor(ImagingSonarSensor_Config * cfg);
};

class ImagingSonarSensor_ROSPublisher : public ROSPublisherInterface {
  ImagingSonarSensor * dev;
public:
  ImagingSonarSensor_ROSPublisher(ImagingSonarSensor *dev, std::string topic, int rate) :
    ROSPublisherInterface(topic, rate), dev(dev) {
  }

  void createPublisher(ros::NodeHandle &nh);
  void publish();

  ~ImagingSonarSensor_ROSPublisher() {
  }
};

#endif
