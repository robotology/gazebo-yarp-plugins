/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

// yarp
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/math/FrameTransform.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

// std
#include <string>

class GazeboYarpModelPosePublisherExample : public yarp::os::RFModule
{   
private:
    /**
     * PolyDriver required to access a yarp::dev::IFrameTransform
     */
    yarp::dev::PolyDriver m_drvTransformClient;

    /**
     * Pointer to yarp::dev::IFrameTransform view of the PolyDriver
     */
    yarp::dev::IFrameTransform* m_tfClient;
    
public:
    /*
     * Configures the module.
     */
    bool configure(yarp::os::ResourceFinder &rf)
    {
	// Prepare properties for the FrameTransformClient
	yarp::os::Property propTfClient;
	propTfClient.put("device", "transformClient");
	propTfClient.put("local", "/transformClientExample");
	propTfClient.put("remote", "/transformServer");
	
	// Try to open the driver
	bool ok_open = m_drvTransformClient.open(propTfClient);
	if (!ok_open) {
	    yError() << "Unable to open the FrameTransformClient driver.";
	    return false;
	}

	// Try to retrieve the view
	bool ok_view = m_drvTransformClient.view(m_tfClient);
	if (!ok_view || m_tfClient == 0) {
	    yError() << "Unable to retrieve the FrameTransformClient view.";
	    return false;
	}

        return true;
    }
    
    /*
     * Defines the cleanup behavior.
     */
    bool close()
    {
	// Close the driver
	return m_drvTransformClient.close();
    }

    /*
     * Defines the period of the module.
     */
    double getPeriod()
    {
	// Update every second
        return 1.0;
    }

    /*
     * Defines the behavior of the module.
     */
    bool updateModule()
    {
	// Check if the module should stop
	if (isStopping())
	    return false;

	// Get the current pose of the model box
	yarp::sig::Matrix matrixTransform(4, 4);
	std::string source = "/inertial";
	std::string target = "/box/frame";
	bool ok_transform = m_tfClient->getTransform(target, source, matrixTransform);
	if (!ok_transform) {
	    // Transform not ready
	    // Wait for the next module update
	    return true;
	}

	// Extract the translational and rotational part
	yarp::math::FrameTransform frameTransform;
	frameTransform.fromMatrix(matrixTransform);
	yarp::sig::Vector pos(3, 0.0);
	pos[0] = frameTransform.translation.tX;
	pos[1] = frameTransform.translation.tY;
	pos[2] = frameTransform.translation.tZ;	
	yarp::sig::Vector rot = frameTransform.getRPYRot();

	// Print transform
	yInfo() << "Transform from" << source
		<< "to" << target << ":";
	yInfo()	<< "Position: " << pos.toString();
	yInfo()	<< "Rotation (Euler ZYX): " << rot.toString();
	yInfo() << "";
	
        return true;
    }
};

int main(int argc, char **argv)
{
    // Check for availability of yarp
    yarp::os::Network yarp;
    if (!yarp.checkNetwork()) {
        yError() << "YARP doesn't seem to be available";
        return 1;
    }

    // Instantiate the example
    GazeboYarpModelPosePublisherExample example;

    // Run the module
    yarp::os::ResourceFinder rf;    
    bool outcome = example.runModule(rf);

    return outcome;
}

