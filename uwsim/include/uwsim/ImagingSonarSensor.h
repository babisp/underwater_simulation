#ifndef ImagingSonarSensor_ECHO_H_
#define ImagingSonarSensor_ECHO_H_

#include "SimulatedDevice.h"
#include <ros/ros.h>
#include "ConfigXMLParser.h"
#include "ROSInterface.h"
#include <std_msgs/Int32.h>
#include "VirtualCamera.h"

#include <osg/Node>
#include <osg/Group>
#include "SimulatedIAUV.h"
#include <osg/PositionAttitudeTransform>

using namespace uwsim;

class ImagingSonarSensor_Config : public SimulatedDeviceConfig
{
public:
  //XML members
  std::string relativeTo;
  double position[3], orientation[3], initAngleX, finalAngleX, initAngleY, finalAngleY, angleIncr, range;
  int visible;
  //constructor
  ImagingSonarSensor_Config(std::string type_) :
    SimulatedDeviceConfig(type_) {}
};

class ImagingSonarSensor_Factory : public SimulatedDeviceFactory
{
public:
  //this is the only place the device/interface type is set
  ImagingSonarSensor_Factory(std::string type_ = "ImagingSonarSensor") :
    SimulatedDeviceFactory(type_) {};

  SimulatedDeviceConfig::Ptr processConfig(const xmlpp::Node* node, ConfigFile * config);
  bool applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *sceneBuilder, size_t iteration);
  std::vector<boost::shared_ptr<ROSInterface> > getInterface(ROSInterfaceInfo & rosInterface,
      std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile);
};

class ImagingSonarSensor : public SimulatedDevice
{

  // struct Remap
  struct Remap2D // use 4 pixels and bilinear interpolation instead
  {
    int x1, x2, y1, y2;
    double weightX1Y1, weightX1Y2, weightX2Y1, weightX2Y2;
    double distort;
  };

public:
  osg::Node *parent;
  std::vector<std::vector<VirtualCamera> > vcams; //Virtual Cameras
  std::string parentLinkName;
  int numpixels, numpixelsX, numpixelsY, camPixelsX, camPixelsY, nCamsX, nCamsY, visible;
  std::string relativeTo;
  double position[3], orientation[3], initAngleX, finalAngleX, initAngleY, finalAngleY, angleIncr, range, camsFOVx, camsFOVy;
  std::vector<std::vector<Remap2D> > remapVector; // make the remap vector 2D
  osg::Node *trackNode;

  osg::ref_ptr<osg::Geode> geode; //Geometry node that draws the beam

  ImagingSonarSensor(ImagingSonarSensor_Config * cfg, osg::Node *trackNode, osg::Group *uwsim_root, unsigned int mask);
  void preCalcTable();
};

class ImagingSonarSensor_ROSPublisher : public ROSPublisherInterface
{
  ImagingSonarSensor * dev;
public:
  ImagingSonarSensor_ROSPublisher(ImagingSonarSensor *dev, std::string topic, int rate) :
    ROSPublisherInterface(topic, rate), dev(dev) {}

  void createPublisher(ros::NodeHandle &nh);
  void publish();

  ~ImagingSonarSensor_ROSPublisher() {}
};

#endif
