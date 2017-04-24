#include <pluginlib/class_list_macros.h>
#include <uwsim/ImagingSonarSensor.h>

ImagingSonarSensor::ImagingSonarSensor(ImagingSonarSensor_Config * cfg, osg::Node *trackNode, osg::Group *uwsim_root, unsigned int mask) :
	SimulatedDevice(cfg)
{
	this->name = cfg->name;
	this->relativeTo = cfg->relativeTo;
	this->visible = cfg->visible;
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

	this->parent = trackNode;

	this->numpixelsX = fabs(finalAngleX - initAngleX) / angleIncr + 1;
	this->numpixelsY = fabs(finalAngleY - initAngleY) / angleIncr + 1;
	this->numpixels = numpixelsX * numpixelsY;

	//Decide number of cameras to use -> using a single camera when the fov is greater than 160 an eyefish distortion appears,...
	// fov 180 doesn't work fov>180 is a completely mess. SO we use cameras until 120 fov and merge the result.
	nCamsX = (int)(finalAngleX - initAngleX) / 120.00000001 + 1;
	nCamsY = (int)(finalAngleY - initAngleY) / 120.00000001 + 1;
	camsFOVx = (finalAngleX - initAngleX) / nCamsX;
	camsFOVy = (finalAngleY - initAngleY) / nCamsY;
	camPixelsX = camsFOVx / angleIncr + 1;
	camPixelsY = camsFOVy / angleIncr + 1;

	for (int i = 0; i < nCamsX; i++)
	{
		std::vector<VirtualCamera> temp;
		for (int j = 0; j < nCamsY; j++)
		{
			// find the rotation of the camera
			osg::PositionAttitudeTransform * mTc = new osg::PositionAttitudeTransform;
			mTc->setPosition(osg::Vec3d(0, 0, 0));
			double angleX = M_PI / 2 - (initAngleX + camsFOVx / 2  + camsFOVx * i) * M_PI / 180.0; // TODO
			double angleY = (initAngleY + camsFOVy / 2  + camsFOVy * j) * M_PI / 180.0;
			mTc->setAttitude(osg::Quat(angleX, osg::Vec3d(1, 0, 0), angleY, osg::Vec3d(0, 1, 0), 0, osg::Vec3d(0, 0, 1)));

			trackNode->asTransform()->addChild(mTc);

			temp.push_back(VirtualCamera(uwsim_root, name, relativeTo, mTc, camPixelsX, camPixelsY, camsFOVx, camPixelsX / camPixelsY, range));

		}
		vcams.push_back(temp);
	}

	preCalcTable();

	// TODO particle filter ?
	// for (int i = 0; i < nCams; i++)
	// {
	// 	vcams[i].textureCamera->setCullMask(mask);
	// }

	// visible beams in simulation
	if (visible)
	{
		osg::ref_ptr<osg::Geometry> beam = osg::ref_ptr<osg::Geometry>(new osg::Geometry);
		osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;
		for (double initAuxX = initAngleX; initAuxX <= finalAngleX; initAuxX += angleIncr)
			for (double initAuxY = initAngleY; initAuxY <= finalAngleY; initAuxY += angleIncr)
			{
				osg::Vec3d start(0, 0, 0);
				osg::Vec3d end(cos(initAuxX * M_PI / 180.0) * sin(initAuxY * M_PI / 180.0) * range, sin(initAuxX * M_PI / 180.0)*range, cos(initAuxX * M_PI / 180.0)*cos(initAuxY * M_PI / 180.0)*range); // TODO
				points->push_back(start);
				points->push_back(end);
			}
		osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
		color->push_back(osg::Vec4(1.0, 0.0, 0.0, 0.35)); // transparent red beams
		beam->setVertexArray(points.get());
		beam->setColorArray(color.get());
		beam->setColorBinding(osg::Geometry::BIND_OVERALL);
		beam->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, points->size()));
		geode = osg::ref_ptr<osg::Geode>(new osg::Geode());
		geode->addDrawable(beam.get());
		geode->setNodeMask(mask);
	}
	trackNode->asGroup()->addChild(geode);

}

void ImagingSonarSensor::preCalcTable()
{

	int iCam, iCamX, iCamY;

	// initialize a numpixelsX * numpixelsY vector
	// will be accessed by remapVector[x][y]
	std::vector<Remap2D> tempVector = std::vector<Remap2D>(numpixelsY,
	{
		0, 0, 0, 0, // pixel1, pixel2, pixel3, pixel4
		0.0, 0.0, 0.0, 0.0, // weight1, weight2, weight3, weight4
		0.0 // distort
	});
	remapVector.resize(numpixelsX, tempVector);

	int currentX;
	int currentY;
	int firstCurrentX, lastCurrentX;
	double thetaX, thetaY;
	double lastThetaX = 0;
	double lastThetaY = 0;
	double thetaCenterX, thetaCenterY;
	osg::Vec3d first, lastX, lastY, lastXY, centerX, centerY, centerXY;
	osg::Matrix *MVPW;

	currentX = 0;
	for (int i = 0; i < numpixelsX; i++)
	{
		iCamX = i / camPixelsX;
		iCamY = 0;

		firstCurrentX = currentX; // used for border later
		currentY = 0;
		for (int j = 0; j < numpixelsY; j++)
		{
			if (j >= camPixelsY * iCamY) // get camera matrices
			{
				//Create matrix to unproject camera points to real world
				MVPW = new osg::Matrix(
				  vcams[iCamX][iCamY].textureCamera->getViewMatrix() * vcams[iCamX][iCamY].textureCamera->getProjectionMatrix()
				  * vcams[iCamX][iCamY].textureCamera->getViewport()->computeWindowMatrix());
				MVPW->invert(*MVPW);

				//Get first last and center points from camera
				first = osg::Vec3d(0, 0, 1) * (*MVPW) ;
				lastX = osg::Vec3d(camPixelsX - 1, 0, 1) * (*MVPW);
				lastY = osg::Vec3d(0, camPixelsY - 1, 1) * (*MVPW);
				lastXY = osg::Vec3d(camPixelsX - 1, camPixelsY - 1, 1) * (*MVPW);
				centerX = osg::Vec3d(camPixelsX / 2, 0, 1) * (*MVPW);
				centerY = osg::Vec3d(0, camPixelsY / 2, 1) * (*MVPW);
				centerXY = osg::Vec3d(camPixelsX / 2, camPixelsY / 2, 0) * (*MVPW);
				thetaCenterX = acos((first * centerX) / (centerX.length() * first.length())) + camsFOVx * iCamX * M_PI / 180;
				thetaCenterY = acos((first * centerY) / (centerY.length() * first.length())) + camsFOVy * iCamY * M_PI / 180;
				iCamY++;
			}

			//Interpolate points
			osg::Vec3d pointX = osg::Vec3d(i % camPixelsX, 0, 1) * (*MVPW);
			osg::Vec3d pointY = osg::Vec3d(0, j % camPixelsY, 1) * (*MVPW);
			osg::Vec3d point = osg::Vec3d(i % camPixelsX, j % camPixelsY, 1) * (*MVPW);

			thetaX = acos(max( min( (first * pointX) / (first.length() * pointX.length()), 1.0), -1.0)) + camsFOVx * (iCamX - 1) * M_PI / 180;
			thetaY = acos(max( min( (first * pointY) / (first.length() * pointY.length()), 1.0), -1.0)) + camsFOVy * (iCamY - 1) * M_PI / 180;
			double cosThetaDistort = fabs( max( min( (centerXY * point) / (centerXY.length() * point.length()), 1.0), -1.0));

			int initialCurrentY = currentY;
			while (thetaX >= angleIncr * currentX * M_PI / 180 && currentX < numpixelsX) // repeat for all points within this area
			{
				currentY = initialCurrentY;
				while (thetaY >= angleIncr * currentY * M_PI / 180 && currentY < numpixelsY)
				{
					// set the remap values
					if (thetaX == angleIncr * currentX * M_PI / 180 or currentX == 0)
					{
						if (thetaY == angleIncr * currentY * M_PI / 180 or currentY == 0)
						{
							// sinlge x, single y
							remapVector[currentX][currentY].x1 = i;
							remapVector[currentX][currentY].x2 = i;
							remapVector[currentX][currentY].y1 = j;
							remapVector[currentX][currentY].y2 = j;

							remapVector[currentX][currentY].weightX1Y1 = 1;
							remapVector[currentX][currentY].weightX1Y2 = 0;
							remapVector[currentX][currentY].weightX2Y1 = 0;
							remapVector[currentX][currentY].weightX2Y2 = 0;
						}
						else
						{
							// sinlge x, two ys
							remapVector[currentX][currentY].x1 = i;
							remapVector[currentX][currentY].x2 = i;
							remapVector[currentX][currentY].y1 = j;
							remapVector[currentX][currentY].y2 = j - 1;

							double dist = fabs(thetaY - angleIncr * currentY * M_PI / 180 ), prevdist = fabs(lastThetaY - angleIncr * currentY * M_PI / 180 );
							double total = prevdist + dist;
							remapVector[currentX][currentY].weightX1Y1 = prevdist / total;
							remapVector[currentX][currentY].weightX1Y2 = dist / total;
							remapVector[currentX][currentY].weightX2Y1 = 0;
							remapVector[currentX][currentY].weightX2Y2 = 0;
						}
					}
					else
					{
						if (thetaY == angleIncr * currentY * M_PI / 180 or currentY == 0)
						{
							// two xs, single y
							remapVector[currentX][currentY].x1 = i;
							remapVector[currentX][currentY].x2 = i - 1;
							remapVector[currentX][currentY].y1 = j;
							remapVector[currentX][currentY].y2 = j;

							double dist = fabs(thetaX - angleIncr * currentX * M_PI / 180 ), prevdist = fabs(lastThetaX - angleIncr * currentX * M_PI / 180 );
							double total = prevdist + dist;
							remapVector[currentX][currentY].weightX1Y1 = prevdist / total;
							remapVector[currentX][currentY].weightX1Y2 = 0;
							remapVector[currentX][currentY].weightX2Y1 = dist / total;
							remapVector[currentX][currentY].weightX2Y2 = 0;
						}
						else
						{
							// two xs, two ys
							remapVector[currentX][currentY].x1 = i;
							remapVector[currentX][currentY].x2 = i - 1;
							remapVector[currentX][currentY].y1 = j;
							remapVector[currentX][currentY].y2 = j - 1;

							double distX = fabs(thetaX - angleIncr * currentX * M_PI / 180 ),
							       prevdistX = fabs(lastThetaX - angleIncr * currentX * M_PI / 180 ),
							       distY = fabs(lastThetaY - angleIncr * currentY * M_PI / 180 ),
							       prevdistY = fabs(lastThetaY - angleIncr * currentY * M_PI / 180 );
							double totalArea = (distX + prevdistX) * (distY + prevdistY);
							remapVector[currentX][currentY].weightX1Y1 = (prevdistX * prevdistY) / totalArea;
							remapVector[currentX][currentY].weightX1Y2 = (prevdistX * distY) / totalArea;
							remapVector[currentX][currentY].weightX2Y1 = (distX * prevdistY) / totalArea;
							remapVector[currentX][currentY].weightX2Y2 = (distX * distY) / totalArea;
						}
					}
					remapVector[currentX][currentY].distort = 1 / cosThetaDistort;

					currentY++;
				}
				currentX++;
			}

			lastThetaX = thetaX;
			lastThetaY = thetaY;
		} // end for j

		lastCurrentX = currentX; // already incremented by 1 in line 235

		for (int tempX = firstCurrentX; tempX < lastCurrentX; tempX++)   // fix all bottom borders
		{
			osg::Vec3d pointX = osg::Vec3d(i % camPixelsX, 0, 1) * (*MVPW);
			osg::Vec3d pointY = osg::Vec3d(0, (numpixelsY - 1) % camPixelsY, 1) * (*MVPW);
			osg::Vec3d point = osg::Vec3d(i % camPixelsX, (numpixelsY - 1) % camPixelsY, 1) * (*MVPW);

			thetaX = acos(max( min( (first * pointX) / (first.length() * pointX.length()), 1.0), -1.0)) + camsFOVx * (iCamX - 1) * M_PI / 180;
			thetaY = acos(max( min( (first * pointY) / (first.length() * pointY.length()), 1.0), -1.0)) + camsFOVy * (iCamY - 1) * M_PI / 180;
			double cosThetaDistort = fabs( max( min( (centerXY * point) / (centerXY.length() * point.length()), 1.0), -1.0));

			remapVector[tempX][numpixelsY - 1].x1 = i;
			remapVector[tempX][numpixelsY - 1].x2 = i - 1;
			remapVector[tempX][numpixelsY - 1].y1 = numpixelsY - 1;
			remapVector[tempX][numpixelsY - 1].y2 = numpixelsY - 1;

			double dist = fabs(thetaX - angleIncr * tempX * M_PI / 180 ), prevdist = fabs(lastThetaX - angleIncr * tempX * M_PI / 180 );
			double total = prevdist + dist;
			remapVector[tempX][numpixelsY - 1].weightX1Y1 = prevdist / total;
			remapVector[tempX][numpixelsY - 1].weightX1Y2 = 0;
			remapVector[tempX][numpixelsY - 1].weightX2Y1 = dist / total;
			remapVector[tempX][numpixelsY - 1].weightX2Y2 = 0;

			remapVector[tempX][numpixelsY - 1].distort = 1 / cosThetaDistort;
		}

	}

	iCamY = 0;
	for (int tempY = 0; tempY < numpixelsY; tempY++) // fix all side borders
	{
		if (tempY >= camPixelsY * iCamY) // get camera matrices
		{
			//Create matrix to unproject camera points to real world
			MVPW = new osg::Matrix(
			  vcams[nCamsX - 1][iCamY].textureCamera->getViewMatrix() * vcams[nCamsX - 1][iCamY].textureCamera->getProjectionMatrix()
			  * vcams[nCamsX - 1][iCamY].textureCamera->getViewport()->computeWindowMatrix());
			MVPW->invert(*MVPW);

			//Get first last and center points from camera
			first = osg::Vec3d(0, 0, 1) * (*MVPW) ;
			lastX = osg::Vec3d(camPixelsX - 1, 0, 1) * (*MVPW);
			lastY = osg::Vec3d(0, camPixelsY - 1, 1) * (*MVPW);
			lastXY = osg::Vec3d(camPixelsX - 1, camPixelsY - 1, 1) * (*MVPW);
			centerX = osg::Vec3d(camPixelsX / 2, 0, 1) * (*MVPW);
			centerY = osg::Vec3d(0, camPixelsY / 2, 1) * (*MVPW);
			centerXY = osg::Vec3d(camPixelsX / 2, camPixelsY / 2, 0) * (*MVPW);
			thetaCenterX = acos((first * centerX) / (centerX.length() * first.length())) + camsFOVx * (nCamsX - 1) * M_PI / 180;
			thetaCenterY = acos((first * centerY) / (centerY.length() * first.length())) + camsFOVy * iCamY * M_PI / 180;
			iCamY++;
		}

		osg::Vec3d pointX = osg::Vec3d((numpixelsX - 1) % camPixelsX, 0, 1) * (*MVPW);
		osg::Vec3d pointY = osg::Vec3d(0, tempY % camPixelsY, 1) * (*MVPW);
		osg::Vec3d point = osg::Vec3d((numpixelsX - 1) % camPixelsX, tempY % camPixelsY, 1) * (*MVPW);

		thetaX = acos(max( min( (first * pointX) / (first.length() * pointX.length()), 1.0), -1.0)) + camsFOVx * (nCamsX - 2) * M_PI / 180;
		thetaY = acos(max( min( (first * pointY) / (first.length() * pointY.length()), 1.0), -1.0)) + camsFOVy * (iCamY - 1) * M_PI / 180;
		double cosThetaDistort = fabs( max( min( (centerXY * point) / (centerXY.length() * point.length()), 1.0), -1.0));

		remapVector[numpixelsX - 1][tempY].x1 = numpixelsX - 1;
		remapVector[numpixelsX - 1][tempY].x2 = numpixelsX - 1;
		remapVector[numpixelsX - 1][tempY].y1 = tempY;
		remapVector[numpixelsX - 1][tempY].y2 = tempY - 1;

		double dist = fabs(thetaY - angleIncr * tempY * M_PI / 180 ), prevdist = fabs(lastThetaY - angleIncr * tempY * M_PI / 180 );
		double total = prevdist + dist;
		remapVector[numpixelsX - 1][tempY].weightX1Y1 = prevdist / total;
		remapVector[numpixelsX - 1][tempY].weightX1Y2 = dist / total;
		remapVector[numpixelsX - 1][tempY].weightX2Y1 = 0;
		remapVector[numpixelsX - 1][tempY].weightX2Y2 = 0;

		remapVector[numpixelsX - 1][tempY].distort = 1 / cosThetaDistort;
	}

	// bottom side corner
	osg::Vec3d pointX = osg::Vec3d((numpixelsX - 1) % camPixelsX, 0, 1) * (*MVPW);
	osg::Vec3d pointY = osg::Vec3d(0, (numpixelsY - 1) % camPixelsY, 1) * (*MVPW);
	osg::Vec3d point = osg::Vec3d((numpixelsX - 1) % camPixelsX, (numpixelsY - 1) % camPixelsY, 1) * (*MVPW);

	thetaX = acos(max( min( (first * pointX) / (first.length() * pointX.length()), 1.0), -1.0)) + camsFOVx * (nCamsX - 2) * M_PI / 180;
	thetaY = acos(max( min( (first * pointY) / (first.length() * pointY.length()), 1.0), -1.0)) + camsFOVy * (nCamsY - 2) * M_PI / 180;
	double cosThetaDistort = fabs( max( min( (centerXY * point) / (centerXY.length() * point.length()), 1.0), -1.0));

	remapVector[numpixelsX - 1][numpixelsY - 1].x1 = numpixelsX - 1;
	remapVector[numpixelsX - 1][numpixelsY - 1].x2 = numpixelsX - 1;
	remapVector[numpixelsX - 1][numpixelsY - 1].y1 = numpixelsY - 1;
	remapVector[numpixelsX - 1][numpixelsY - 1].y2 = numpixelsY - 1;

	remapVector[numpixelsX - 1][numpixelsY - 1].weightX1Y1 = 1;
	remapVector[numpixelsX - 1][numpixelsY - 1].weightX1Y2 = 0;
	remapVector[numpixelsX - 1][numpixelsY - 1].weightX2Y1 = 0;
	remapVector[numpixelsX - 1][numpixelsY - 1].weightX2Y2 = 0;

	remapVector[numpixelsX - 1][numpixelsY - 1].distort = 1 / cosThetaDistort;

	// for (int i = 0; i < nCamsX; i++)
	// 	for (int j = 0; j < nCamsY; j++)
	// 	{
	// 		iCam = i * nCamsY + j; // find which camera to use

	// 		// Create matrix to unproject camera points to real world
	// 		MVPW = new osg::Matrix(
	// 		  vcams[iCam].textureCamera->getViewMatrix() * vcams[iCam].textureCamera->getProjectionMatrix()
	// 		  * vcams[iCam].textureCamera->getViewport()->computeWindowMatrix());
	// 		MVPW->invert(*MVPW);

	// 		// Get first last and center points from camera
	// 		first = osg::Vec3d(0, 0, 1) * (*MVPW) ;
	// 		last = osg::Vec3d(camPixelsX - 1, camPixelsY - 1, 1) * (*MVPW);
	// 		center = osg::Vec3d(camPixelsX / 2, camPixelsY / 2, 1) * (*MVPW);
	// 		thetacenter = acos((first * center) / (center.length() * first.length())) + camsFOV * iCam * M_PI / 180;

	// 		// now find the margins of the pixels in the remap vector
	// 		for (int ii = i * camPixelsX; ii < (i + 1) * camPixelsX; ii++)
	// 			for (int jj = j * camPixelsY; jj < (j + 1) * camPixelsY; jj++)
	// 			{
	// 				//
	// 			}
	// 	}
}

SimulatedDeviceConfig::Ptr ImagingSonarSensor_Factory::processConfig(const xmlpp::Node* node, ConfigFile * config)
{
	ImagingSonarSensor_Config * cfg = new ImagingSonarSensor_Config(getType());
	xmlpp::Node::NodeList list = node->get_children();
	for (xmlpp::Node::NodeList::iterator iter = list.begin(); iter != list.end(); ++iter)
	{
		const xmlpp::Node* child = dynamic_cast<const xmlpp::Node*>(*iter);
		if (child->get_name() == "relativeTo")
			config->extractStringChar(child, cfg->relativeTo);
		else if (child->get_name() == "position")
			config->extractPositionOrColor(child, cfg->position);
		else if (child->get_name() == "visible")
			config->extractIntChar(child, cfg->visible);
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

bool ImagingSonarSensor_Factory::applyConfig(SimulatedIAUV * auv, Vehicle &vehicleChars, SceneBuilder *sceneBuilder, size_t iteration)
{
	if (iteration > 0)
		return true;
	for (size_t i = 0; i < vehicleChars.simulated_devices.size(); ++i)
	{
		if (vehicleChars.simulated_devices[i]->getType() == this->getType())
		{
			ImagingSonarSensor_Config * cfg = dynamic_cast<ImagingSonarSensor_Config *>(vehicleChars.simulated_devices[i].get());
			if (cfg)
			{
				int target = -1;
				for (int j = 0; j < auv->urdf->link.size(); j++)
				{
					if (auv->urdf->link[j]->getName() == cfg->relativeTo)
					{
						target = j;
					}
				}
				if (target == -1)
				{
					OSG_FATAL << "ImagingSonarSensor device '" << vehicleChars.simulated_devices[i]->name << "' inside robot '"
					          << vehicleChars.name << "' has an unknown relativeTo, discarding..." << std::endl;
				}
				else
				{
					osg::ref_ptr < osg::Transform > vMd = (osg::Transform*) new osg::PositionAttitudeTransform;
					vMd->asPositionAttitudeTransform()->setPosition(osg::Vec3d(cfg->position[0], cfg->position[1], cfg->position[2]));
					vMd->asPositionAttitudeTransform()->setAttitude(
					  osg::Quat(cfg->orientation[0], osg::Vec3d(1, 0, 0), cfg->orientation[1], osg::Vec3d(0, 1, 0),
					            cfg->orientation[2], osg::Vec3d(0, 0, 1)));
					auv->urdf->link[target]->getParent(0)->getParent(0)->asGroup()->addChild(vMd);

					unsigned int mask = sceneBuilder->scene->getOceanScene()->getNormalSceneMask(); // TODO allow option in XML
					auv->devices->all.push_back(ImagingSonarSensor::Ptr(new ImagingSonarSensor(cfg, vMd, sceneBuilder->root, mask)));
				}
			}
			else
				OSG_FATAL << "ImagingSonarSensor device '" << vehicleChars.simulated_devices[i]->name << "' inside robot '"
				          << vehicleChars.name << "' has empty cfg, discarding..." << std::endl;
		}
	}
	return true;
}

std::vector<boost::shared_ptr<ROSInterface> > ImagingSonarSensor_Factory::getInterface(ROSInterfaceInfo & rosInterface, std::vector<boost::shared_ptr<SimulatedIAUV> > & iauvFile)
{
	std::vector < boost::shared_ptr<ROSInterface> > ifaces;
	for (size_t i = 0; i < iauvFile.size(); ++i)
	{
		for (size_t d = 0; d < iauvFile[i]->devices->all.size(); ++d)
		{
			if (iauvFile[i]->devices->all[d]->getType() == this->getType()
			    && iauvFile[i]->devices->all[d]->name == rosInterface.targetName)
			{
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

void ImagingSonarSensor_ROSPublisher::createPublisher(ros::NodeHandle &nh)
{
	ROS_INFO("ImagingSonarSensor_ROSPublisher on topic %s", topic.c_str());
	pub_ = nh.advertise < sensor_msgs::MultiEchoLaserScan > (topic, 1);
}

void ImagingSonarSensor_ROSPublisher::publish()
{
	sensor_msgs::MultiEchoLaserScan msg;

	msg.header.stamp = getROSTime();
	msg.header.frame_id = this->name;

	msg.angle_min = initAngleX;
	msg.angle_max = finalAngleX;
	msg.angle_increment = angleIncr;

	msg.range_min = 1.0;
	msg.range_max = range;

	// TODO read distance values

	pub_.publish(msg);
}

#if ROS_VERSION_MINIMUM(1, 9, 0)
// new pluginlib API in Groovy and Hydro
PLUGINLIB_EXPORT_CLASS(ImagingSonarSensor_Factory, uwsim::SimulatedDeviceFactory)
#else
PLUGINLIB_REGISTER_CLASS(ImagingSonarSensor_Factory, ImagingSonarSensor_Factory, uwsim::SimulatedDeviceFactory)
#endif
