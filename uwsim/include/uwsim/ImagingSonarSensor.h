#ifndef IMAGING_SONAR_SENSOR_ECHO_H_
#define IMAGING_SONAR_SENSOR_ECHO_H_
#include "SimulatedDevice.h"
#include "ConfigXMLParser.h"
#include <ros/ros.h>
#include <osg/Node>
#include "SimulatedIAUV.h"
#include <osg/PositionAttitudeTransform>
using namespace uwsim;

class ImagingSonarSensor_Config : public SimulatedDeviceConfig {
public:
  //XML members
  std::string relativeTo;
  double position[3], orientation[3], initAngleX, finalAngleX, initAngleY, finalAngleY, angleIncr, range;
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
  osg::Node *parent;
  double initAngleX, finalAngleX, initAngleY, finalAngleY, angleIncr, range;

  ImagingSonarSensor(ImagingSonarSensor_Config *cfg, osg::Node *trackNode);
};

#endif
