#include <pluginlib/class_list_macros.h>
#include <uwsim/ImagingSonarSensor.h>

/* You will need to add your code HERE */

SimulatedDeviceConfig::Ptr ImagingSonarSensor_Factory::processConfig(const xmlpp::Node* node, ConfigFile * config) {
  ImagingSonarSensor_Config * cfg = new ImagingSonarSensor_Config(getType());
  xmlpp::Node::NodeList list = node->get_children();
  for (xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter) {
    const xmlpp::Node* child = dynamic_cast<const xmlpp::Node*>(*iter);
    if (child->get_name() == "relativeTo")
      config->extractStringChar(child, cfg->relativeTo);
    else if (child->get_name() == "position")
      config->extractPositionOrColor(child, cfg->position);
    else if (child->get_name() == "orientation")
      config->extractOrientation(child, cfg->orientation);
    else if (child->getName() == "initAngleX")
      config->extractFloatChar(child, cfg->initAngleX);
    else if (child->getName() == "finalAngleX")
      config->extractFloatChar(child, cfg->finalAngleX);
    else if (child->getName() == "initAngleY")
      config->extractFloatChar(child, cfg->initAngleY);
    else if (child->getName() == "finalAngleY")
      config->extractFloatChar(child, cfg->finalAngleY);
    else if (child->getName() == "angleIncr")
      config->extractFloatChar(child, cfg->angleIncr);
    else if (child->getName() == "range")
      config->extractFloatChar(child, cfg->range);
  }

  return SimulatedDeviceConfig::Ptr(cfg);
}

bool ImagingSonarSensor_Factory::applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *sceneBuilder, size_t iteration) {

}

std::vector<boost::shared_ptr<ROSInterface> > ImagingSonarSensor_Factory::getInterface(ROSInterfaceInfo & rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile) {

}

#if ROS_VERSION_MINIMUM(1, 9, 0)
// new pluginlib API in Groovy and Hydro
PLUGINLIB_EXPORT_CLASS(FACTORYCLASSNAME_Factory, uwsim::SimulatedDeviceFactory)
#else
PLUGINLIB_REGISTER_CLASS(FACTORYCLASSNAME_Factory, FACTORYCLASSNAME_Factory, uwsim::SimulatedDeviceFactory)
#endif
