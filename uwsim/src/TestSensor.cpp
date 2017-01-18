#include <pluginlib/class_list_macros.h>
#include <uwsim/TestSensor.h>

/* You will need to add your code HERE */

SimulatedDeviceConfig::Ptr TestSensor_Factory::processConfig(const xmlpp::Node* node, ConfigFile * config){}

bool TestSensor_Factory::applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *sceneBuilder, size_t iteration){}

std::vector<boost::shared_ptr<ROSInterface> > TestSensor_Factory::getInterface(ROSInterfaceInfo & rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile){}


#if ROS_VERSION_MINIMUM(1, 9, 0)
// new pluginlib API in Groovy and Hydro
PLUGINLIB_EXPORT_CLASS(FACTORYCLASSNAME_Factory, uwsim::SimulatedDeviceFactory)
#else
PLUGINLIB_REGISTER_CLASS(FACTORYCLASSNAME_Factory, FACTORYCLASSNAME_Factory, uwsim::SimulatedDeviceFactory)
#endif
