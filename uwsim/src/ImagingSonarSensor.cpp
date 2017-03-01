#include <pluginlib/class_list_macros.h>
#include <uwsim/ImagingSonarSensor.h>

ImagingSonarSensor::ImagingSonarSensor(ImagingSonarSensor_Config * cfg) :
	SimulatedDevice(cfg) {
	this->frequency = cfg->frequency;
}

SimulatedDeviceConfig::Ptr ImagingSonarSensor_Factory::processConfig(const xmlpp::Node* node, ConfigFile * config) {
	ImagingSonarSensor_Config * cfg = new ImagingSonarSensor_Config(getType());
	xmlpp::Node::NodeList list = node->get_children();
	for (xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter) {
		const xmlpp::Node* child = dynamic_cast<const xmlpp::Node*>(*iter);
		// if (child->get_name() == "relativeTo")
		//   config->extractStringChar(child, cfg->relativeTo);
		// else if (child->get_name() == "position")
		//   config->extractPositionOrColor(child, cfg->position);
		// else if (child->get_name() == "orientation")
		//   config->extractOrientation(child, cfg->orientation);
		if (child->get_name() == "frequency")
			config->extractIntChar(child, cfg->frequency);
	}

	return SimulatedDeviceConfig::Ptr(cfg);
}

bool ImagingSonarSensor_Factory::applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *sceneBuilder, size_t iteration) {
	if (iteration > 0)
		return true;
	for (size_t i = 0; i < vehicleChars.simulated_devices.size(); ++i) {
		if (vehicleChars.simulated_devices[i]->getType() == this->getType()) {
			ImagingSonarSensor_Config * cfg = dynamic_cast<ImagingSonarSensor_Config *>(vehicleChars.simulated_devices[i].get());
			if (cfg) {
				auv->devices->all.push_back(ImagingSonarSensor::Ptr(new ImagingSonarSensor(cfg)));
			} else
				OSG_FATAL << "ImagingSonarSensor device '" << vehicleChars.simulated_devices[i]->name << "' inside robot '"
				          << vehicleChars.name << "' has empty cfg, discarding..." << std::endl;
		}
	}
	return true;
}

std::vector<boost::shared_ptr<ROSInterface> > ImagingSonarSensor_Factory::getInterface(ROSInterfaceInfo & rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile) {
	std::vector < boost::shared_ptr<ROSInterface> > ifaces;
	for (size_t i = 0; i < iauvFile.size(); ++i) {
		for (size_t d = 0; d < iauvFile[i]->devices->all.size(); ++d) {
			if (iauvFile[i]->devices->all[d]->getType() == this->getType()
			    && iauvFile[i]->devices->all[d]->name == rosInterface.targetName) {
				ifaces.push_back(
				  boost::shared_ptr<ROSInterface>(new ImagingSonarSensor_ROSPublisher(dynamic_cast<ImagingSonarSensor*>(iauvFile[i]->devices->all[d].get()),
				                                  rosInterface.topic, rosInterface.rate)));
			}
		}
	}
	if (ifaces.size() == 0)
		ROS_WARN("Returning empty ROS interface for device %s...", rosInterface.targetName.c_str());
	return ifaces;
}

void ImagingSonarSensor_ROSPublisher::createPublisher(ros::NodeHandle &nh) {
	ROS_INFO("ImagingSonarSensor_ROSPublisher on topic %s", topic.c_str());
	pub_ = nh.advertise < std_msgs::Int32 > (topic, 1);
}

void ImagingSonarSensor_ROSPublisher::publish() {
	std_msgs::Int32 msg;
	msg.data = dev->frequency;
	pub_.publish(msg);
}

#if ROS_VERSION_MINIMUM(1, 9, 0)
// new pluginlib API in Groovy and Hydro
PLUGINLIB_EXPORT_CLASS(ImagingSonarSensor_Factory, uwsim::SimulatedDeviceFactory)
#else
PLUGINLIB_REGISTER_CLASS(ImagingSonarSensor_Factory, ImagingSonarSensor_Factory, uwsim::SimulatedDeviceFactory)
#endif
