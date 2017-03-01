#ifndef IMAGING_SONAR_SENSOR_ECHO_H_
#define IMAGING_SONAR_SENSOR_ECHO_H_
#include "SimulatedDevice.h"
#include <ros/ros.h>
using namespace uwsim;

/* You will need to add your code HERE */
class ImagingSonarSensor_Factory : public SimulatedDeviceFactory {
public:
  //this is the only place the device/interface type is set
  ImagingSonarSensor_Factory(std::string type_ = "ImagingSonarSensor") :
    SimulatedDeviceFactory(type_) {
  };

  SimulatedDeviceConfig::Ptr processConfig(const xmlpp::Node* node, ConfigFile * config);
  bool applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *sceneBuilder, size_t iteration);
  std::vector<boost::shared_ptr<ROSInterface>> getInterface(ROSInterfaceInfo & rosInterface,
      std::vector<boost::shared_ptr<SimulatedIAUV>> & iauvFile);
};

#endif
