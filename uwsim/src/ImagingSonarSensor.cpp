#include <pluginlib/class_list_macros.h>
#include <uwsim/ImagingSonarSensor.h>

#define MAX_CAM_FOV 120.00000001
// #define MAX_CAM_FOV 60.00000001
// #define MAX_CAM_FOV 30.00000001

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
	nCamsX = (int)(finalAngleX - initAngleX) / MAX_CAM_FOV + 1;
	nCamsY = (int)(finalAngleY - initAngleY) / MAX_CAM_FOV + 1;
	camsFOVx = (finalAngleX - initAngleX) / nCamsX;
	camsFOVy = (finalAngleY - initAngleY) / nCamsY;
	camPixelsX = camsFOVx / angleIncr + 1;
	camPixelsY = camsFOVy / angleIncr + 1;

	for (int i = 0; i < nCamsX; i++)
	{
		double angleX = (initAngleX + camsFOVx / 2  + camsFOVx * i) * M_PI / 180.0;
		std::vector<VirtualCamera> temp;
		for (int j = 0; j < nCamsY; j++)
		{
			// find the rotation of the camera
			osg::PositionAttitudeTransform * mTc = new osg::PositionAttitudeTransform;
			mTc->setPosition(osg::Vec3d(0, 0, 0));
			double angleY = (initAngleY + camsFOVy / 2  + camsFOVy * (nCamsY - 1 - j)) * M_PI / 180.0; // TODO: for some strange reason the cameras must be created in the reverse order on this axis
			mTc->setAttitude(osg::Quat(angleX, osg::Vec3d(1, 0, 0), angleY, osg::Vec3d(0, 1, 0), 0, osg::Vec3d(0, 0, 1)));

			trackNode->asTransform()->addChild(mTc);

			temp.push_back(VirtualCamera(uwsim_root, name, relativeTo, mTc, camPixelsX, camPixelsY, camsFOVx, camPixelsX / (float) camPixelsY, range));

		}
		vcams.push_back(temp);
	}

	preCalcTable();

	// particle filter
	for (int i = 0; i < nCamsX; i++)
		for (int j = 0; j < nCamsY; j++)
			vcams[i][j].textureCamera->setCullMask(mask);

	// visible beams in simulation
	if (visible)
	{
		osg::ref_ptr<osg::Geometry> beam = osg::ref_ptr<osg::Geometry>(new osg::Geometry);
		osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;
		for (double initAuxX = initAngleX; initAuxX <= finalAngleX; initAuxX += angleIncr)
			for (double initAuxY = initAngleY; initAuxY <= finalAngleY; initAuxY += angleIncr)
			{
				osg::Vec3d start(0, 0, 0);
				osg::Vec3d end(-cos(initAuxX * M_PI / 180.0) * sin(initAuxY * M_PI / 180.0) * range, sin(initAuxX * M_PI / 180.0)*range, -cos(initAuxX * M_PI / 180.0)*cos(initAuxY * M_PI / 180.0)*range);
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
	double thetaX, thetaY;
	double lastThetaX = 0;
	double lastThetaY = 0;
	double thetaCenterX, thetaCenterY;
	osg::Vec3d first, centerX, centerY, centerXY;
	osg::Matrix *MVPW;

	currentX = 0;
	for (int i = 0; i < numpixelsX; i++)
	{
		iCamX = i / camPixelsX;

		MVPW = new osg::Matrix(
		  vcams[iCamX][0].textureCamera->getViewMatrix() * vcams[iCamX][0].textureCamera->getProjectionMatrix()
		  * vcams[iCamX][0].textureCamera->getViewport()->computeWindowMatrix());
		MVPW->invert(*MVPW);

		//Get first last and center points from camera
		first = osg::Vec3d(0, 0, 1) * (*MVPW) ;
		centerX = osg::Vec3d(camPixelsX / 2, 0, 1) * (*MVPW);

		osg::Vec3d pointX = osg::Vec3d(i % camPixelsX, 0, 1) * (*MVPW);
		thetaX = acos(max( min( (first * pointX) / (first.length() * pointX.length()), 1.0), -1.0)) + camsFOVx * iCamX * M_PI / 180;

		while (thetaX >= angleIncr * currentX * M_PI / 180 && currentX < numpixelsX) // repeat for all points within this area
		{
			currentY = 0;
			iCamY = 0;
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

				thetaX = acos(max( min( (first * pointX) / (first.length() * pointX.length()), 1.0), -1.0)) + camsFOVx * iCamX * M_PI / 180; // iCamX is already lower by 1
				thetaY = acos(max( min( (first * pointY) / (first.length() * pointY.length()), 1.0), -1.0)) + camsFOVy * (iCamY - 1) * M_PI / 180;
				double cosThetaDistort = fabs( max( min( (centerXY * point) / (centerXY.length() * point.length()), 1.0), -1.0));

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
				lastThetaY = thetaY;
			} // end for j

			// fix right edge
			osg::Vec3d point = osg::Vec3d(i % camPixelsX, (numpixelsY - 1) % camPixelsY, 1) * (*MVPW);
			double cosThetaDistort = fabs( max( min( (centerXY * point) / (centerXY.length() * point.length()), 1.0), -1.0));
			if (thetaX == angleIncr * currentX * M_PI / 180 or currentX == 0)
			{
				// single x
				remapVector[currentX][(numpixelsY - 1)].x1 = i;
				remapVector[currentX][(numpixelsY - 1)].x2 = i;
				remapVector[currentX][(numpixelsY - 1)].y1 = (numpixelsY - 1);
				remapVector[currentX][(numpixelsY - 1)].y2 = (numpixelsY - 1) - 1;

				double dist = fabs(thetaY - angleIncr * (numpixelsY - 1) * M_PI / 180 ), prevdist = fabs(lastThetaY - angleIncr * (numpixelsY - 1) * M_PI / 180 );
				double total = prevdist + dist;
				remapVector[currentX][(numpixelsY - 1)].weightX1Y1 = 0.5;
				remapVector[currentX][(numpixelsY - 1)].weightX1Y2 = 0.5;
				remapVector[currentX][(numpixelsY - 1)].weightX2Y1 = 0;
				remapVector[currentX][(numpixelsY - 1)].weightX2Y2 = 0;
			}
			else
			{
				// two xs
				remapVector[currentX][(numpixelsY - 1)].x1 = i;
				remapVector[currentX][(numpixelsY - 1)].x2 = i - 1;
				remapVector[currentX][(numpixelsY - 1)].y1 = (numpixelsY - 1);
				remapVector[currentX][(numpixelsY - 1)].y2 = (numpixelsY - 1) - 1;

				double distX = fabs(thetaX - angleIncr * currentX * M_PI / 180 ),
				       prevdistX = fabs(lastThetaX - angleIncr * currentX * M_PI / 180 ),
				       distY = 0.5,
				       prevdistY = 0.5;
				double totalArea = (distX + prevdistX) * (distY + prevdistY);
				remapVector[currentX][(numpixelsY - 1)].weightX1Y1 = (prevdistX * prevdistY) / totalArea;
				remapVector[currentX][(numpixelsY - 1)].weightX1Y2 = (prevdistX * distY) / totalArea;
				remapVector[currentX][(numpixelsY - 1)].weightX2Y1 = (distX * prevdistY) / totalArea;
				remapVector[currentX][(numpixelsY - 1)].weightX2Y2 = (distX * distY) / totalArea;
			}
			remapVector[currentX][(numpixelsY - 1)].distort = 1 / cosThetaDistort;

			currentX++;
		}
		lastThetaX = thetaX;
	}

	// fix bottom edge
	currentY = 0;
	int i = currentX = numpixelsX - 1;
	iCamX = i / camPixelsX;
	iCamY = 0;
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

		thetaX = acos(max( min( (first * pointX) / (first.length() * pointX.length()), 1.0), -1.0)) + camsFOVx * iCamX * M_PI / 180; // iCamX is already lower by 1
		thetaY = acos(max( min( (first * pointY) / (first.length() * pointY.length()), 1.0), -1.0)) + camsFOVy * (iCamY - 1) * M_PI / 180;
		double cosThetaDistort = fabs( max( min( (centerXY * point) / (centerXY.length() * point.length()), 1.0), -1.0));

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
		lastThetaY = thetaY;
	} // end for j

	// bottom right corner
	osg::Vec3d point = osg::Vec3d((numpixelsX - 1) % camPixelsX, (numpixelsY - 1) % camPixelsY, 1) * (*MVPW);
	double cosThetaDistort = fabs(max( min( (centerXY * point) / (centerXY.length() * point.length()), 1.0), -1.0));

	remapVector[numpixelsX - 1][numpixelsY - 1].x1 = numpixelsX - 1;
	remapVector[numpixelsX - 1][numpixelsY - 1].x2 = numpixelsX - 1;
	remapVector[numpixelsX - 1][numpixelsY - 1].y1 = numpixelsY - 1;
	remapVector[numpixelsX - 1][numpixelsY - 1].y2 = numpixelsY - 1;

	remapVector[numpixelsX - 1][numpixelsY - 1].weightX1Y1 = 1;
	remapVector[numpixelsX - 1][numpixelsY - 1].weightX1Y2 = 0;
	remapVector[numpixelsX - 1][numpixelsY - 1].weightX2Y1 = 0;
	remapVector[numpixelsX - 1][numpixelsY - 1].weightX2Y2 = 0;

	remapVector[numpixelsX - 1][numpixelsY - 1].distort = 1 / cosThetaDistort;
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
					ImagingSonarSensor* tempSensor = new ImagingSonarSensor(cfg, vMd, sceneBuilder->root, mask);
					auv->devices->all.push_back(ImagingSonarSensor::Ptr(tempSensor));

					// add all the virtual cameras of the new sensor in the vehicle's camview vector so that they can be manipulated by the SceneBuilder
					for (unsigned int i = 0; i < tempSensor->nCamsX; i++)
						for (unsigned int j = 0; j < tempSensor->nCamsY; j++)
							auv->camview.push_back(tempSensor->vcams[i][j]);
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
	if (dev != NULL)
	{
		sensor_msgs::MultiEchoLaserScan msg;

		msg.header.stamp = getROSTime();
		msg.header.frame_id = dev->name;

		msg.angle_min = dev->initAngleX * M_PI / 180;
		msg.angle_max = dev->finalAngleX * M_PI / 180;
		msg.angle_increment = dev->angleIncr * M_PI / 180;

		msg.range_min = 1.0;
		msg.range_max = dev->range;

		double fov, aspect, near, far;

		std::vector<std::vector<double> > tmp;
		std::vector<double> tmp1 = std::vector<double>(dev->camPixelsY * dev->nCamsY, 0.0);
		tmp.resize(dev->camPixelsX * dev->nCamsX, tmp1);

		int offsetX = 0;
		for (unsigned int jX = 0; jX < dev->nCamsX ; jX++)
		{
			int offsetY = 0;
			for (unsigned int jY = 0; jY < dev->nCamsY ; jY++)
			{
				dev->vcams[jX][jY].textureCamera->getProjectionMatrixAsPerspective(fov, aspect, near, far);

				float * data = (float *) dev->vcams[jX][jY].depthTexture->data();
				double a = far / (far - near);
				double b = (far * near) / (near - far);


				for (int i = 0; i < dev->camPixelsX; i++)
					for (int j = 0; j < dev->camPixelsY; j++)
					{
						double Z = data[i * dev->camPixelsX + j];
						tmp[i + offsetX][j + offsetY] = b / (Z - a);
					}

				offsetY += dev->camPixelsY;
			}
			offsetX += dev->camPixelsX;
		}

		sensor_msgs::LaserEcho laserEchoMsg;
		laserEchoMsg.echoes.resize(dev->numpixelsX, 0.0);
		msg.ranges.resize(dev->numpixelsY, laserEchoMsg);
		for (int i = 0; i < dev->numpixelsY; i++)
		{
			for (int j = 0; j < dev->numpixelsX; j++)
			{
				ImagingSonarSensor::Remap2D remap = dev->remapVector[j][i]; // the world axis are different from the image axis
				msg.ranges[i].echoes[j] = (tmp[remap.x1][remap.y1] * remap.weightX1Y1
				                           + tmp[remap.x1][remap.y2] * remap.weightX1Y2
				                           + tmp[remap.x2][remap.y1] * remap.weightX2Y1
				                           + tmp[remap.x2][remap.y2] * remap.weightX2Y2) * remap.distort;
				if (msg.ranges[i].echoes[j] > dev->range)
					msg.ranges[i].echoes[j] = dev->range;
			}
		}

		pub_.publish(msg);
	}
}

#if ROS_VERSION_MINIMUM(1, 9, 0)
// new pluginlib API in Groovy and Hydro
PLUGINLIB_EXPORT_CLASS(ImagingSonarSensor_Factory, uwsim::SimulatedDeviceFactory)
#else
PLUGINLIB_REGISTER_CLASS(ImagingSonarSensor_Factory, ImagingSonarSensor_Factory, uwsim::SimulatedDeviceFactory)
#endif
