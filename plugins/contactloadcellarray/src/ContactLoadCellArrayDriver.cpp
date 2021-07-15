/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "ContactLoadCellArrayDriver.h"
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

#include <sdf/Param.hh>

#define DEBUG 0

using namespace yarp::dev;

GazeboYarpContactLoadCellArrayDriver::GazeboYarpContactLoadCellArrayDriver()
{

}

GazeboYarpContactLoadCellArrayDriver::~GazeboYarpContactLoadCellArrayDriver()
{

}

void GazeboYarpContactLoadCellArrayDriver::onUpdate(const gazebo::common::UpdateInfo& _info)
{
    const gazebo::msgs::Contacts& contacts = this->m_sensor->Contacts();

    ignition::math::Vector3d resultantForceonCG(0.0, 0.0, 0.0);
    ignition::math::Vector3d resultantMomentonCG(0.0, 0.0, 0.0);

    const double TIME_CONVERSION_NS = 1.0e-9;
    double timestamp = -1;
    for (size_t i = 0; i < contacts.contact_size(); i++)
    {
        for (int j = 0; j < contacts.contact(i).position_size(); j++)
        {
            double t = contacts.contact(i).time().sec() + TIME_CONVERSION_NS*contacts.contact(i).time().nsec();
            if (timestamp < 0)
            {
                timestamp = t;
            }

            // This block allows addition of wrench components only if they arrive in same timestamp
            if (!ignition::math::equal(t, timestamp))
                continue;

            const gazebo::msgs::JointWrench& wrench = contacts.contact(i).wrench(j);

            // we are interested only in the link specific contact wrenches and not the contact surface
            if (wrench.body_1_name() == this->m_linkCollisionName)
            {
                resultantForceonCG += ignition::math::Vector3d(wrench.body_1_wrench().force().x(),
                                                            wrench.body_1_wrench().force().y(),
                                                            wrench.body_1_wrench().force().z());
                resultantMomentonCG += ignition::math::Vector3d(wrench.body_1_wrench().torque().x(),
                                                             wrench.body_1_wrench().torque().y(),
                                                             wrench.body_1_wrench().torque().z());
            }
            else if (wrench.body_2_name() == this->m_linkCollisionName)
            {
                resultantForceonCG += ignition::math::Vector3d(wrench.body_2_wrench().force().x(),
                                                            wrench.body_2_wrench().force().y(),
                                                            wrench.body_2_wrench().force().z());
                resultantMomentonCG += ignition::math::Vector3d(wrench.body_2_wrench().torque().x(),
                                                             wrench.body_2_wrench().torque().y(),
                                                             wrench.body_2_wrench().torque().z());
            }
        }
    }

#if DEBUG
     std::cout << "Wrench at CG [ " << resultantForceonCG.X() << " " <<  resultantForceonCG.Y() << " " << resultantForceonCG.Z() << " " << resultantMomentonCG.X() << " " <<  resultantMomentonCG.Y() << " " << resultantMomentonCG.Z() << " ]" << std::endl;
#endif

    // The wrench is applied at the CG and with the rotation same as link origin
    // consider L: Link Origin Frame, C: Link CG Frame, ^: cross product operator givng a skew symmetric matrix
    // L_f = C_f
    // L_m = [L_o_c]^L_f
    ignition::math::Vector3d resultantForceonLinkOrigin = resultantForceonCG;

    ignition::math::Matrix3d L_o_C_cross = ignition::math::Matrix3d(0, -m_linkOrigin_Pos_linkCG.Z(), m_linkOrigin_Pos_linkCG.Y(),
                                                                    m_linkOrigin_Pos_linkCG.Z(), 0, -m_linkOrigin_Pos_linkCG.X(),
                                                                    -m_linkOrigin_Pos_linkCG.Y(), m_linkOrigin_Pos_linkCG.X(), 0);
    ignition::math::Vector3d resultantMomentonLinkOrigin = resultantMomentonCG + L_o_C_cross*resultantForceonLinkOrigin;

#if DEBUG
    std::cout << "Wrench at Link Origin [ " << resultantForceonLinkOrigin.X() << " " <<  resultantForceonLinkOrigin.Y() << " " << resultantForceonLinkOrigin.Z() << " " << resultantMomentonLinkOrigin.X() << " " <<  resultantMomentonLinkOrigin.Y() << " " << resultantMomentonLinkOrigin.Z() << " ]" << std::endl;
#endif

    // build a reduced wrench only with normal force f_{z}, and tangential moments tau_{x}, tau_{y}
    yarp::sig::Vector reducedWrench;
    reducedWrench.clear();
    reducedWrench.push_back(resultantForceonLinkOrigin.Z());
    reducedWrench.push_back(resultantMomentonLinkOrigin.X());
    reducedWrench.push_back(resultantMomentonLinkOrigin.Y());

    // map this reduced wrench to the load cell normal forces at the respective load cell locations
    if (reducedWrench.size() != m_mapWrenchtoNormalForce.cols())
    {
        yError() << "GazeboYarpContactLoadCellArrayDriver Error: Improper contact wrench dimensions. cannot map to load cell normal forces";
    }
    m_contactNormalForces.resize(m_mapWrenchtoNormalForce.rows());
    using namespace yarp::math;
    m_contactNormalForces = m_mapWrenchtoNormalForce*reducedWrench;

    // This flag is important. If its not set to true, the analog sensor ouput will be zero
    m_dataAvailable = true;
    m_stamp.update(m_sensor->LastMeasurementTime().Double());

#if DEBUG
    checkCoP(reducedWrench);
#endif
}

void GazeboYarpContactLoadCellArrayDriver::checkCoP(const yarp::sig::Vector& threeAxisContactForceTorque)
{
    yarp::sig::Vector copFromEquivalentContactWrench(2);
    copFromEquivalentContactWrench[0] = -threeAxisContactForceTorque[2]/threeAxisContactForceTorque[0]; // -tau_{y]/f_{z}
    copFromEquivalentContactWrench[1] = threeAxisContactForceTorque[1]/threeAxisContactForceTorque[0];  // tau_{x}/f_{z}


    // Computing COP from contact normal forces, assumption load cell locations are on the same XY plane
    double sumOfNormalForcesInN = 0.0;
    yarp::sig::Vector weightedSumOfLocations(3);
    weightedSumOfLocations.zero();

    // using namespace yarp::math;
    using namespace yarp::sig;
    for (size_t i = 0; i < m_contactNormalForces.size(); i++)
    {
        sumOfNormalForcesInN += m_contactNormalForces[i];
        weightedSumOfLocations += m_contactNormalForces[i]*m_loadCellLocations[i];
    }

    yarp::sig::Vector copFromContactNormalForces(2);
    copFromContactNormalForces[0] = weightedSumOfLocations[0]/sumOfNormalForcesInN;
    copFromContactNormalForces[1] = weightedSumOfLocations[1]/sumOfNormalForcesInN;

    yarp::sig::Vector differenceInCoP(2);
    differenceInCoP = copFromEquivalentContactWrench - copFromContactNormalForces;

#if DEBUG
    //yInfo() << "GazeboYarpContactLoadCellArrayDriver: Measured Wrench: [" << threeAxisContactForceTorque[0] <<", " << threeAxisContactForceTorque[1] << ", " << threeAxisContactForceTorque[2] << "]";
    //yInfo() << "GazeboYarpContactLoadCellArrayDriver: Measured CoP: [" << copFromEquivalentContactWrench[0] <<", " << copFromEquivalentContactWrench[1] <<"]";
    //yInfo() << "GazeboYarpContactLoadCellArrayDriver: Estimated CoP: [" << copFromContactNormalForces[0] <<", " << copFromContactNormalForces[1] <<"]";
    yInfo() << "GazeboYarpContactLoadCellArrayDriver: Difference in measured CoP and estimated CoP: [" << differenceInCoP[0] <<", " << differenceInCoP[1] <<"]";
#endif

    if (std::abs(copFromContactNormalForces[0] - copFromEquivalentContactWrench[0])/copFromEquivalentContactWrench[0] > 1e-09)
    {
        yError() << "GazeboYarpContactLoadCellArrayDriver: Measured CoP and Estiamted CoP X intercept do not match";
    }

    if (std::abs(copFromContactNormalForces[1] - copFromEquivalentContactWrench[1])/copFromEquivalentContactWrench[1] > 1e-09)
    {
        yError() << "GazeboYarpContactLoadCellArrayDriver: Measured CoP and Estiamted CoP Y intercept do not match";
    }
}


bool GazeboYarpContactLoadCellArrayDriver::open(yarp::os::Searchable& config)
{
    yarp::os::Property pluginParams;
    pluginParams.fromString(config.toString().c_str());

    std::string robotName(pluginParams.find("robotName").asString().c_str());

    this->m_robot = GazeboYarpPlugins::Handler::getHandler()->getRobot(robotName);
    if (this->m_robot == NULL)
    {
        yError() << "GazeboYarpContactLoadCellArrayDriver: Robot Model was not found";
        return false;
    }

    if (!this->initLinkAssociatedToContactSensor(pluginParams))
    {
        return false;
    }

    if (!this->initContactSensor())
    {
        return false;
    }

    if (!this->configure(pluginParams))
    {
        return false;
    }

    if (!this->prepareLinkInformation())
    {
        return false;
    }

    std::lock_guard<std::mutex> guard(m_dataMutex);

    this->m_updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboYarpContactLoadCellArrayDriver::onUpdate, this, _1));
    this->m_sensor->SetActive(true);

    return true;
}

bool GazeboYarpContactLoadCellArrayDriver::close()
{
    std::lock_guard<std::mutex> guard(m_dataMutex);
    this->m_updateConnection.reset();
    return true;
}

bool GazeboYarpContactLoadCellArrayDriver::initLinkAssociatedToContactSensor(yarp::os::Property &pluginParameters)
{
    m_linkAssociateToSensor = pluginParameters.find("linkName").asString();

    const gazebo::physics::Link_V &gazeboModelLinks = m_robot->GetLinks();
    std::string linkNameScopedEnding = "::" + m_linkAssociateToSensor;
    bool linkFound = false;
    for (size_t gazeboLink = 0; gazeboLink < gazeboModelLinks.size(); gazeboLink++)
    {
        std::string gazeboLinkName = gazeboModelLinks[gazeboLink]->GetScopedName();
        if (GazeboYarpPlugins::hasEnding(gazeboLinkName, linkNameScopedEnding))
        {
            linkFound = true;
            m_sensorLink = boost::get_pointer(gazeboModelLinks[gazeboLink]);
            break;
        }
    }

    if (!linkFound)
    {
        yError() << "GazeboYarpContactLoadCellArrayDriver: initContactSensor(): Link associated to sensor not found";
        return false;
    }

    return true;
}

bool GazeboYarpContactLoadCellArrayDriver::initContactSensor()
{
    // Choosing to get the sensor information from the sdf::Element of the link rather than gazebo::LinkPtr
    // because Gazebo has a division between physics engine and sensors engine handled in
    // separate threads. To avoid any runtime discrepancy, the decision to get info from sdf is made.
    // Also the Link Element might have multiple sensors, we need specifically the contact type sensor
    // Assumes there is only one contact collision element per link
    size_t linkSensorCount = m_sensorLink->GetSensorCount();
    sdf::ElementPtr sensorsdf;
    bool contactSensorFound = false;
    for (size_t sensor = 0; sensor < linkSensorCount; sensor++)
    {
        sensorsdf = (m_sensorLink->GetSDF()).get()->GetElement("sensor");
        if (sensorsdf.get()->GetAttribute("type")->GetAsString() == "contact")
        {
            break;
        }
    }

    std::string sensorName = sensorsdf.get()->GetAttribute("name")->GetAsString();
    // For some reason scoped name is not necessary force
    // instantiating the contact sensor, uncomment the block if necessary
    // Get collision name
    sdf::ElementPtr collision = (sensorsdf.get()->GetElement("contact")).get()->GetElement("collision");
    std::string collisionName = collision.get()->GetValue()->GetAsString();
    /* std::string tmp = m_sensorLink->GetName() + "::" + collisionName + "::" + sensorName;
     * sensorName.clear();
     * sensorName = tmp;
     */


    this->m_linkCollisionName = m_robot->GetName() + "::" + m_sensorLink->GetName() + "::" + collisionName;

    std::lock_guard<std::mutex> guard(m_dataMutex);
    gazebo::sensors::SensorManager *mgr = gazebo::sensors::SensorManager::Instance();

    // If sensors are not initialized, fail.
    if (!mgr->SensorsInitialized())
    {
        yError() << "GazeboYarpContactLoadCellArrayDriver: Sensors not initialized";
        return false;
    }


    gazebo::sensors::SensorPtr contact = mgr->GetSensor(sensorName);
    this->m_sensor = dynamic_cast<gazebo::sensors::ContactSensor*>(contact.get());
    if (this->m_sensor == nullptr)
    {
        yError() << "GazeboYarpContactLoadCellArrayDriver: Could not get pointer to the sensor";
        return false;
    }
    this->m_sensor->SetActive(true);
    return true;
}


bool GazeboYarpContactLoadCellArrayDriver::configure(yarp::os::Property& pluginParams)
{
    // Get load cell locations with respect to the CG of the link (body1)
    yarp::os::Bottle *loadCellNames = pluginParams.find("loadCellNames").asList();
    if (loadCellNames->size() == 0)
    {
        yError() << "GazeboYarpContactLoadCellArrayDriver: Error parsing parameters: \"loadCellNames\" should be followed by  list";
        return false;
    }

    yarp::os::Bottle *loadCellX = pluginParams.find("loadCellX").asList();
    yarp::os::Bottle *loadCellY = pluginParams.find("loadCellY").asList();
    yarp::os::Bottle *loadCellZ = pluginParams.find("loadCellZ").asList();

    if (loadCellX->size() == 0 || loadCellY->size() == 0 || loadCellZ->size() == 0)
    {
        yError() << "GazeboYarpContactLoadCellArrayDriver: Error parsing parameters: \"loadCellX , ..Y, ..Z\" should be followed by  list";
        return false;
    }

    if (loadCellX->size() != loadCellNames->size() || loadCellY->size() != loadCellNames->size() || loadCellZ->size() != loadCellNames->size())
    {
        yError() << "GazeboYarpContactLoadCellArrayDriver: Error parsing parameters: \"loadCellX , ..Y, ..Z\" should be the same size as \"loadCellNames\"";
        return false;
    }

    for (size_t i = 0; i < loadCellNames->size(); i++)
    {
        yarp::sig::Vector loadCellLoc(3);
        loadCellLoc.clear();
        loadCellLoc.push_back(loadCellX->get(i).asFloat64());
        loadCellLoc.push_back(loadCellY->get(i).asFloat64());
        loadCellLoc.push_back(loadCellZ->get(i).asFloat64());
        this->m_loadCellLocations.push_back(loadCellLoc);
     }

     m_contactNormalForces.resize(m_loadCellLocations.size());
     if (!this->prepareMappingMatrix())
     {
         return false;
     }

    return true;
}

bool GazeboYarpContactLoadCellArrayDriver::prepareMappingMatrix()
{
    // get the size of the load cell locations
    int n_cells = this->m_loadCellLocations.size();
    const int N_AXIS = 3;

    // Consider w = Af ; where w is the wrench of size 3x1 containing normal force, tangential and radial torques
    // A is the matrix that maps the array of load cell normal forces f to an resultant wrench w acting
    // at the contact. However, we ae solving the inverse problem of mapping a wrench acting at the contact link
    // to the presure sensor array normal forces, hence we need to obtain the pseudo inverse through
    // f = (A+)w ; where (A+) is the pseudo inverse. Mapping an equivalent wrench to a number of load cell
    // normal forces has infinite solutions, but considering the pseudo inverse gives us the solution with minimized norm.
    // Hence it is not necessary that the array of load cell simulate the same forces at corresponding locations
    // similar to the real robot. However, the resultant wrench from the load cell array of normal forces should be valid.
    // The matrix A is the function of loadCell locations (x, y) with respect to the link coordinate frame

    // ASSUMPTION: The load cells are distributed over a plane perpendicular to the z-axis of the link frame
    // and the load cells are measuring the force over the z-direction
    yarp::sig::Matrix A;
    A.resize(N_AXIS ,n_cells);

    // Add the respective columns to the mapping matrix
    for (int i = 0; i < n_cells; i++)
    {
        yarp::sig::Vector columnMap;
        columnMap.clear();
        columnMap.push_back(1);
        columnMap.push_back(this->m_loadCellLocations[i][1]); // r_y : y-intercept of the contact point with respect to link origin
        columnMap.push_back(-this->m_loadCellLocations[i][0]); // -r_x : x-intercept of the contact point with respect to link origin

        if (!A.setCol(i, columnMap))
        {
            yError() << "GazeboYarpContactLoadCellArrayDriver Failed: Failed to form mapping matrix";
            return false;
        }
    }

    if (A.rows() != N_AXIS || A.cols() != m_loadCellLocations.size())
    {
        yError() << "GazeboYarpContactLoadCellArrayDriver Failed: Mismatched dimensions in the mapping matrix";
        return false;
    }

    // Compute the pseudo-inverse to determine the mapping matrix
    this->m_mapWrenchtoNormalForce = yarp::math::pinv(A);

    return true;
}

bool GazeboYarpContactLoadCellArrayDriver::prepareLinkInformation()
{
    // \url https://bitbucket.org/osrf/gazebo/issues/545/request-change-contact-reference-frame
    // \url https://bitbucket.org/osrf/gazebo/pull-requests/355/bullet-contact-sensor/diff
    // According to the above issue and PR, the force torque feedback values are acting at the
    // CG with respect to the link origin frame.

#if GAZEBO_MAJOR_VERSION >= 8
    // Just get information about location of link CG with respect to the link origin frame
    this->m_linkOrigin_Pos_linkCG = m_sensorLink->GetInertial().get()->Pose().Pos();
#else
    gazebo::math::Pose tmp = m_sensorLink->GetInertial().get()->GetPose();
    this->m_linkOrigin_Pos_linkCG = ignition::math::Vector3d(tmp.pos.x, tmp.pos.y, tmp.pos.z);
#endif

#if DEBUG
     std::cout << "L_o_C: \n [ " << m_linkOrigin_Pos_linkCG[0] << " " <<  m_linkOrigin_Pos_linkCG[1] << " " << m_linkOrigin_Pos_linkCG[2] << " ]"<< std::endl;
#endif

    return true;
}

int GazeboYarpContactLoadCellArrayDriver::read(yarp::sig::Vector& out)
{
    std::lock_guard<std::mutex> guard(m_dataMutex);

    if (!m_dataAvailable)
    {
        return AS_TIMEOUT;
    }

    out.resize(m_contactNormalForces.size());
    out = m_contactNormalForces;

    return AS_OK;
}


yarp::os::Stamp GazeboYarpContactLoadCellArrayDriver::getLastInputStamp()
{
    return this->m_stamp;
}


int GazeboYarpContactLoadCellArrayDriver::getState(int ch)
{
    return AS_OK;
}

int GazeboYarpContactLoadCellArrayDriver::calibrateChannel(int ch)
{
    return AS_OK;
}

int GazeboYarpContactLoadCellArrayDriver::calibrateChannel(int ch, double v)
{
    return AS_OK;
}

int GazeboYarpContactLoadCellArrayDriver::calibrateSensor(const yarp::sig::Vector& value)
{
    return AS_OK;
}

int GazeboYarpContactLoadCellArrayDriver::calibrateSensor()
{
    return AS_OK;
}

int GazeboYarpContactLoadCellArrayDriver::getChannels()
{
    return AS_OK;
}
