#include <pluginlib/class_list_macros.h>
#include <uwsim/TestSensor.h>

TestSensor::TestSensor(TestSensor_Config * cfg) :
SimulatedDevice(cfg)
{
	this->frequency = cfg->frequency;
}

SimulatedDeviceConfig::Ptr TestSensor_Factory::processConfig(const xmlpp::Node* node, ConfigFile * config)
{
	TestSensor_Config * cfg = new TestSensor_Config(getType());
	xmlpp::Node::NodeList list = node->get_children();
	for (xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)
	{
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

bool TestSensor_Factory::applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *sceneBuilder, size_t iteration)
{
	if (iteration > 0)
		return true;
	for (size_t i = 0; i < vehicleChars.simulated_devices.size(); ++i)
	{
		if (vehicleChars.simulated_devices[i]->getType() == this->getType())
		{
			TestSensor_Config * cfg = dynamic_cast<TestSensor_Config *>(vehicleChars.simulated_devices[i].get());
			if (cfg)
			{
				auv->devices->all.push_back(TestSensor::Ptr(new TestSensor(cfg)));
			}
			else
				OSG_FATAL << "TestSensor device '" << vehicleChars.simulated_devices[i]->name << "' inside robot '"
			<< vehicleChars.name << "' has empty cfg, discarding..." << std::endl;
		}
	}
	return true;
}

std::vector<boost::shared_ptr<ROSInterface> > TestSensor_Factory::getInterface(ROSInterfaceInfo & rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile){}

#if ROS_VERSION_MINIMUM(1, 9, 0)
// new pluginlib API in Groovy and Hydro
PLUGINLIB_EXPORT_CLASS(TestSensor_Factory, uwsim::SimulatedDeviceFactory)
#else
PLUGINLIB_REGISTER_CLASS(TestSensor_Factory, TestSensor_Factory, uwsim::SimulatedDeviceFactory)
#endif
