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
    else if (child->get_name() == "initAngleX")
      config->extractFloatChar(child, cfg->initAngleX);
    else if (child->get_name() == "finalAngleX")
      config->extractFloatChar(child, cfg->finalAngleX);
    else if (child->get_name() == "initAngleY")
      config->extractFloatChar(child, cfg->initAngleY);
    else if (child->get_name() == "finalAngleY")
      config->extractFloatChar(child, cfg->finalAngleY);
    else if (child->get_name() == "angleIncr")
      config->extractFloatChar(child, cfg->angleIncr);
    else if (child->get_name() == "range")
      config->extractFloatChar(child, cfg->range);
  }

  return SimulatedDeviceConfig::Ptr(cfg);
}

bool ImagingSonarSensor_Factory::applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *sceneBuilder, size_t iteration) {
  if (iteration > 0)
    return true;
  for (size_t i = 0; i < vehicleChars.simulated_devices.size(); ++i)
    if (vehicleChars.simulated_devices[i]->getType() == this->getType()) {
      ImagingSonarSensor_Config * cfg = dynamic_cast<ImagingSonarSensor_Config *>(vehicleChars.simulated_devices[i].get());
      if (cfg) {
        int target = -1;
        for (int j = 0; j < auv->urdf->link.size(); j++) {
          if (auv->urdf->link[j]->getName() == cfg->relativeTo) {
            target = j;
          }
        }
        if (target == -1) {
          OSG_FATAL << "ImagingSonarSensor device '" << vehicleChars.simulated_devices[i]->name << "' inside robot '"
                    << vehicleChars.name << "' has an unknown relativeTo, discarding..." << std::endl;
        } else {
          osg::ref_ptr < osg::Transform > vMd = (osg::Transform*) new osg::PositionAttitudeTransform;
          vMd->asPositionAttitudeTransform()->setPosition(osg::Vec3d(cfg->position[0], cfg->position[1], cfg->position[2]));
          vMd->asPositionAttitudeTransform()->setAttitude(
            osg::Quat(cfg->orientation[0], osg::Vec3d(1, 0, 0), cfg->orientation[1], osg::Vec3d(0, 1, 0),
                      cfg->orientation[2], osg::Vec3d(0, 0, 1)));
          auv->urdf->link[target]->getParent(0)->getParent(0)->asGroup()->addChild(vMd);
          auv->devices->all.push_back(ImagingSonarSensor::Ptr(new ImagingSonarSensor(cfg, vMd)));
        }
      } else
        OSG_FATAL << "ImagingSonarSensor device '" << vehicleChars.simulated_devices[i]->name << "' inside robot '"
                  << vehicleChars.name << "' has empty cfg, discarding..." << std::endl;
    }
  return true;
}

std::vector<boost::shared_ptr<ROSInterface> > ImagingSonarSensor_Factory::getInterface(ROSInterfaceInfo & rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile) {

}

ImagingSonarSensor::ImagingSonarSensor(ImagingSonarSensor_Config * cfg) :
  SimulatedDevice(cfg) {
  this->relativeTo = cfg->relativeTo;
  this->position[0] = cfg->position[0];
  this->position[1] = cfg->position[1];
  this->position[2] = cfg->position[2];
  this->orientation[0] = cfg->orientation[0];
  this->orientation[1] = cfg->orientation[1];
  this->orientation[2] = cfg->orientation[2];
  this->initAngleX = cfg->initAngleX;
  this->finalAngleX = cfg->finalAngleX;
  this->initAngleY = cfg->initAngleY;
  this->finalAngleY = cfg->finalAngleY;
  this->angleIncr = cfg->angleIncr;
  this->range = cfg->range;
}

ImagingSonarSensor::ImagingSonarSensor(ImagingSonarSensor_Config * cfg, osg::Node *trackNode) :
  SimulatedDevice(cfg) {
  this->parent = trackNode;
  this->initAngleX = cfg->initAngleX;
  this->finalAngleX = cfg->finalAngleX;
  this->initAngleY = cfg->initAngleY;
  this->finalAngleY = cfg->finalAngleY;
  this->angleIncr = cfg->angleIncr;
  this->range = cfg->range;
}

#if ROS_VERSION_MINIMUM(1, 9, 0)
// new pluginlib API in Groovy and Hydro
PLUGINLIB_EXPORT_CLASS(FACTORYCLASSNAME_Factory, uwsim::SimulatedDeviceFactory)
#else
PLUGINLIB_REGISTER_CLASS(FACTORYCLASSNAME_Factory, FACTORYCLASSNAME_Factory, uwsim::SimulatedDeviceFactory)
#endif
