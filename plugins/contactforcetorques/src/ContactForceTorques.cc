/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include <vector>

#include "ContactForceTorques.hh"

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Vector.h>

#include <gazebo/common/Events.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>

#include <ignition/math/Pose3.hh>

GZ_REGISTER_MODEL_PLUGIN(gazebo::GazeboYarpContactForceTorques)

namespace gazebo
{
struct GazeboYarpContactForceTorques::ForceTorque
{
    ignition::math::Vector3d force;
    ignition::math::Vector3d torque;
};


struct GazeboYarpContactForceTorques::AdditionalFrameInformation
{
    std::string additionalFrame_name;
    std::string link_name;
    gazebo::physics::LinkPtr link;
    ignition::math::Pose3d link_H_frame;
};

struct GazeboYarpContactForceTorques::OutputWrenchPortInformation
{
    std::string port_name{""};
    std::string link_name{""};
    gazebo::physics::LinkPtr link;
    std::string orientation_frame{""};
    std::string origin_frame{""};
    int link_index{0};
    int orientation_frame_index{0};
    int origin_frame_index{0};
    yarp::sig::Vector output_vector;
    ignition::math::Pose3d publishingFrame_H_linkFrame;
    yarp::os::BufferedPort<yarp::sig::Vector>* output_port;

    OutputWrenchPortInformation(): output_port(new yarp::os::BufferedPort<yarp::sig::Vector>())
    {
    }

    ~OutputWrenchPortInformation()
    {
        if (output_port) {
            delete output_port;
        }
        output_port = nullptr;
    }
};

struct GazeboYarpContactForceTorques::Pimpl
{
    double publishingPeriodInSeconds {-1.0};
    double latestUpdateInstantInSeconds {-1.0};
    gazebo::physics::ContactManager* contactManager {nullptr};
    std::string robot{""};
    yarp::os::Property config;
    gazebo::event::ConnectionPtr updateConnection;
    gazebo::event::ConnectionPtr resetConnection;
    std::vector<  GazeboYarpContactForceTorques::AdditionalFrameInformation > additionalFrameInfos;
    std::vector<  GazeboYarpContactForceTorques::OutputWrenchPortInformation > outputFTPorts;
};

GazeboYarpContactForceTorques::GazeboYarpContactForceTorques() : ModelPlugin(), m_pimpl(new GazeboYarpContactForceTorques::Pimpl())
{

}

GazeboYarpContactForceTorques::~GazeboYarpContactForceTorques()
{
    GazeboYarpPlugins::Handler::getHandler()->removeRobot(m_pimpl->robot);
    closePorts();
    yarp::os::Network::fini();
}

void GazeboYarpContactForceTorques::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Get  contact manager
    m_pimpl->contactManager = _parent->GetWorld()->Physics()->GetContactManager();


    yarp::os::Network::init();
    if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout))
    {
        yError() << "GazeboYarpContactForceTorques::Load erros: yarp network does not seem to be available, is the yarp server running?";
        return;
    }

    if (!_parent)
    {
        yError() << "GazeboYarpContactForceTorques plugin requires a parent \n";
        return;
    }

    GazeboYarpPlugins::Handler::getHandler()->setRobot(boost::get_pointer(_parent));
    m_pimpl->robot = _parent->GetScopedName();

    // Getting .ini configuration file parameters from sdf
    bool configuration_loaded = GazeboYarpPlugins::loadConfigModelPlugin(_parent, _sdf, m_pimpl->config);

    if (!configuration_loaded) {
        yError() << "GazeboYarpContactForceTorques : File .ini not found, load failed." ;
        return;
    }

    // Try to parse and load parameters
    bool okLoadParams = this->LoadParams(_parent);

    if (!okLoadParams) {
        yError() << "GazeboYarpContactForceTorques : File .ini malformed, load of the plugin  failed." ;
        return;
    }

    // Try to open the YARP ports
    if (!this->openPorts()) {
        return;
    }

    // Create callback
    m_pimpl->updateConnection =
        gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboYarpContactForceTorques::onUpdate,
                this, _1));

    m_pimpl->resetConnection =
        gazebo::event::Events::ConnectWorldReset(boost::bind(&GazeboYarpContactForceTorques::onReset, this));

}

bool GazeboYarpContactForceTorques::LoadAdditionalFrameParams(physics::ModelPtr model)
{
    yarp::os::Bottle portsProp = m_pimpl->config.findGroup("OUTPUT_EXTERNAL_FORCETORQUES_FRAMES");

    // The group is optional
    if (portsProp.isNull())
    {
        return true;
    }

    for(size_t i = 1; i < portsProp.size(); i++)
    {
        GazeboYarpContactForceTorques::AdditionalFrameInformation additionalFrameStruct;
        yarp::os::Bottle *additional_frame = portsProp.get(i).asList();
        if( additional_frame == NULL || additional_frame->isNull() || additional_frame->size() != 2
                || additional_frame->get(1).asList() == NULL
                || !(additional_frame->get(1).asList()->size() == 7) )
        {
            yError() << "GazeboYarpContactForceTorques plugin failed: malformed OUTPUT_EXTERNAL_FORCETORQUES_FRAMES group found in configuration, exiting";
            if (additional_frame)
            {
                yError() << "GazeboYarpContactForceTorques plugin failed: malformed line " << additional_frame->toString();
            }
            else
            {
                yError() << "GazeboYarpContactForceTorques plugin failed: malformed line " << portsProp.get(i).toString();
                yError() << "GazeboYarpContactForceTorques plugin failed: malformed group " << portsProp.toString();
            }
            return false;
        }

        additionalFrameStruct.additionalFrame_name = additional_frame->get(0).asString();
        additionalFrameStruct.link_name = additional_frame->get(1).asList()->get(0).asString();

        // Get the exact link with only link name instead of full_scoped_link_name
        physics::Link_V links = model->GetLinks();
        size_t lnk=0;
        for(lnk=0; lnk < links.size(); lnk++)
        {
            std::string candidate_link_name = links[lnk]->GetScopedName();

            // hasEnding compare ending of the condidate model link name to the given link name, in order to be able to use unscoped names
            if (GazeboYarpPlugins::hasEnding(candidate_link_name, additionalFrameStruct.link_name))
            {
                additionalFrameStruct.link = links[lnk];
                break;
            }
        }

        // If no link was found
        if (lnk == links.size()) {
            yError() << "GazeboYarpContactForceTorques plugin failed: link with name "
                     <<  additionalFrameStruct.link_name << " not found in the model";
            return false;
        }

        double x = additional_frame->get(1).asList()->get(1).asFloat64();
        double y = additional_frame->get(1).asList()->get(2).asFloat64();
        double z = additional_frame->get(1).asList()->get(3).asFloat64();
        double roll = additional_frame->get(1).asList()->get(4).asFloat64();
        double pitch = additional_frame->get(1).asList()->get(5).asFloat64();
        double yaw = additional_frame->get(1).asList()->get(6).asFloat64();

        additionalFrameStruct.link_H_frame =
            ignition::math::Pose3d(x, y, z, roll, pitch, yaw);

        m_pimpl->additionalFrameInfos.push_back(additionalFrameStruct);
    }


    return true;
}

bool GazeboYarpContactForceTorques::LoadStreamingPortParams(physics::ModelPtr model)
{
    yarp::os::Bottle portsProp = m_pimpl->config.findGroup("OUTPUT_EXTERNAL_FORCETORQUES_PORTS");
    if (portsProp.isNull())
    {
        yError() << "GazeboYarpContactForceTorques plugin failed: [OUTPUT_EXTERNAL_FORCETORQUES_PORTS] group not found in config file";
        return false;
    }

    // Load values
    m_pimpl->outputFTPorts.clear();
    m_pimpl->outputFTPorts.resize(portsProp.size()-1);
    for(size_t output_wrench_port = 1; output_wrench_port < portsProp.size(); output_wrench_port++)
    {
        GazeboYarpContactForceTorques::OutputWrenchPortInformation& wrench_port_struct = m_pimpl->outputFTPorts[output_wrench_port-1];
        yarp::os::Bottle *wrench_port = portsProp.get(output_wrench_port).asList();
        if( wrench_port == NULL || wrench_port->isNull() || wrench_port->size() != 2
                || wrench_port->get(1).asList() == NULL
                || !(wrench_port->get(1).asList()->size() == 2 || wrench_port->get(1).asList()->size() == 3 ) )
        {
            yError() << "GazeboYarpContactForceTorques plugin failed: malformed OUTPUT_EXTERNAL_FORCETORQUES_PORTS group found in configuration, exiting";
            if( wrench_port )
            {
                yError() << "GazeboYarpContactForceTorques plugin failed: malformed line " << wrench_port->toString();
            }
            else
            {
                yError() << "GazeboYarpContactForceTorques plugin failed: malformed line " << portsProp.get(output_wrench_port).toString();
                yError() << "GazeboYarpContactForceTorques plugin failed: malformed group " << portsProp.toString();
            }
            return false;
        }

        wrench_port_struct.port_name = wrench_port->get(0).asString();
        wrench_port_struct.link_name = wrench_port->get(1).asList()->get(0).asString();

        // Get the exact link with only link name instead of full_scoped_link_name
        physics::Link_V links = model->GetLinks();
        size_t lnk=0;
        for(lnk=0; lnk < links.size(); lnk++)
        {
            std::string candidate_link_name = links[lnk]->GetScopedName();

            // hasEnding compare ending of the condidate model link name to the given link name, in order to be able to use unscoped names
            if (GazeboYarpPlugins::hasEnding(candidate_link_name, wrench_port_struct.link_name))
            {
                wrench_port_struct.link = links[lnk];
                break;
            }
        }

        // If no link was found
        if (lnk == links.size()) {
            yError() << "GazeboYarpContactForceTorques plugin failed: link with name "
                     << wrench_port_struct.link_name << " not found in the model";
            return false;
        }

        if( wrench_port->get(1).asList()->size() == 2 )
        {
            // Simple configuration, both the origin and the orientation of the
            // force belong to the same frame
            wrench_port_struct.orientation_frame = wrench_port->get(1).asList()->get(1).asString();
            wrench_port_struct.origin_frame = wrench_port_struct.orientation_frame;
        }
        else
        {
            assert(wrench_port->get(1).asList()->size() != 3);
            // Complex configuration: the first parameter is the frame of the point of expression,
            // the second parameter is the frame of orientation
            wrench_port_struct.origin_frame = wrench_port->get(1).asList()->get(1).asString();
            wrench_port_struct.orientation_frame = wrench_port->get(1).asList()->get(2).asString();
        }

        // TODO(traversaro): eventually, support the complete case, for now the following two restriction will hold:
        // * origin_frame and orientation_frame need to match
        // * origin_frame and  orientation_frame can be either the link frame, or an additional frame rigidly attached to the link of interest
        if (wrench_port_struct.origin_frame != wrench_port_struct.orientation_frame) {
            yError() << "GazeboYarpContactForceTorques plugin failed: link with name "
                     << wrench_port_struct.link_name << " requested with different orientation frame and  origin frame, currently not supported.";
            return false;
        }

        // If wrench_port_struct.origin_frame == wrench_port_struct.orientation_frame == wrench_port_struct.link_name, then
        // the publishingFrame_H_linkFrame transform is just the identity
        wrench_port_struct.publishingFrame_H_linkFrame = ignition::math::Pose3d(0, 0, 0, 0, 0, 0);
        if (wrench_port_struct.origin_frame != wrench_port_struct.link_name) {
            bool additionalFrameFound=false;
            for(auto&& additionalFrame: m_pimpl->additionalFrameInfos) {
                if (additionalFrame.additionalFrame_name == wrench_port_struct.origin_frame) {
                    if (additionalFrame.link_name != wrench_port_struct.link_name) {
                        yError() << "GazeboYarpContactForceTorques plugin failed: link with name "
                                 << wrench_port_struct.link_name << " requested with an additional frame that rigidly attached to another link, currently not supported.";
                        return false;
                    }
                    additionalFrameFound = true;
                    wrench_port_struct.publishingFrame_H_linkFrame = additionalFrame.link_H_frame.Inverse();
                }
            }
            if (!additionalFrameFound) {
                yError() << "GazeboYarpContactForceTorques plugin failed: link with name "
                                 << wrench_port_struct.link_name << " requested an additional frame that is not specified.";
                return false;
            }

        }
    }

    return true;
}

bool GazeboYarpContactForceTorques::LoadParams(physics::ModelPtr model)
{
    // First load periodInSeconds
    if (!m_pimpl->config.check("periodInSeconds")) {
        yError() << "GazeboYarpContactForceTorques plugin failed: required parameter periodInSeconds not found in config file";
        return false;
    }

    m_pimpl->publishingPeriodInSeconds = m_pimpl->config.find("periodInSeconds").asFloat64();

    if (m_pimpl->publishingPeriodInSeconds <= 0.0) {
        yError() << "GazeboYarpContactForceTorques plugin failed: required parameter periodInSeconds has unsupported negative or null value.";
        return false;
    }

    // Then load additonal frames data
    if (!this->LoadAdditionalFrameParams(model)) {
        return false;
    }

    // Then load streaming ports data
    if (!this->LoadStreamingPortParams(model)) {
        return false;
    }

    return true;
}

bool GazeboYarpContactForceTorques::openPorts()
{
    for(unsigned int i = 0; i < m_pimpl->outputFTPorts.size(); i++ )
    {
        size_t nrOfChannelsOfYARPFTSensor = 6;
        if (!m_pimpl->outputFTPorts[i].output_port->open(m_pimpl->outputFTPorts[i].port_name)) {
            yError() << "GazeboYarpContactForceTorques plugin failed: impossible to open port "
                     << m_pimpl->outputFTPorts[i].port_name;
            return false;
        }
        m_pimpl->outputFTPorts[i].output_vector.resize(nrOfChannelsOfYARPFTSensor);
    }
    return true;
}

bool GazeboYarpContactForceTorques::closePorts()
{
    for(unsigned int i = 0; i < m_pimpl->outputFTPorts.size(); i++ )
    {
        m_pimpl->outputFTPorts[i].output_port->close();
    }
    return true;
}

/**
 * Return ft_newFrame
 */
GazeboYarpContactForceTorques::ForceTorque
GazeboYarpContactForceTorques::applyTransformToForceTorque(ignition::math::Pose3d& newFrame_H_oldFrame,
                                                           GazeboYarpContactForceTorques::ForceTorque&  ft_oldFrame)
{
    // Implement the same formula of https://robotology.github.io/idyntree/master/classiDynTree_1_1Transform.html#af120c9b238d9efdf947109b47c4e4fdf
    GazeboYarpContactForceTorques::ForceTorque ft_newFrame;
    ignition::math::Quaterniond newFrame_R_oldFrame = newFrame_H_oldFrame.Rot();

    ft_newFrame.force = newFrame_R_oldFrame*ft_oldFrame.force;
    ft_newFrame.torque = newFrame_H_oldFrame.Pos().Cross(ft_newFrame.force) + newFrame_R_oldFrame*ft_oldFrame.torque;

    return ft_newFrame;
}


void GazeboYarpContactForceTorques::toYARP(GazeboYarpContactForceTorques::ForceTorque& ft_gazebo,
                                           yarp::sig::Vector& ft_yarp)
{
    ft_yarp.resize(6);
    ft_yarp[0] = ft_gazebo.force.X();
    ft_yarp[1] = ft_gazebo.force.Y();
    ft_yarp[2] = ft_gazebo.force.Z();
    ft_yarp[3] = ft_gazebo.torque.X();
    ft_yarp[4] = ft_gazebo.torque.Y();
    ft_yarp[5] = ft_gazebo.torque.Z();
    return;
}

GazeboYarpContactForceTorques::ForceTorque GazeboYarpContactForceTorques::getTotalForceTorqueAppliedOnLink(gazebo::physics::LinkPtr link)
{
    // Buffer for total contact force torque of the link, expressed at the link center of mass and with the orientation of the link, initially zero
    // This is done due to:
    // \url https://bitbucket.org/osrf/gazebo/issues/545/request-change-contact-reference-frame
    // \url https://bitbucket.org/osrf/gazebo/pull-requests/355/bullet-contact-sensor/diff
    // According to the above issue and PR, the force torque feedback values are acting at the CG with respect to the link origin frame.
    GazeboYarpContactForceTorques::ForceTorque ft_linkCOM;
    ft_linkCOM.force = ignition::math::Vector3d::Zero;
    ft_linkCOM.torque = ignition::math::Vector3d::Zero;

    // This loop can probably be improved by just looping over the
    // contacts once and computing the total force for all links
    const std::vector< gazebo::physics::Contact* >& contacts = m_pimpl->contactManager->GetContacts();
    for(auto&& contact : contacts) {
        // Collision1 belongs to the link of interested
        if (contact->collision1->GetLink()->GetId() == link->GetId() ) {
            for(int cntIdx=0; cntIdx < contact->count; cntIdx++) {
                ft_linkCOM.force += contact->wrench->body1Force;
                ft_linkCOM.torque += contact->wrench->body1Torque;
            }
        }

        // Collision2 belongs to the link of interested
        if (contact->collision2->GetLink()->GetId() == link->GetId() ) {
            for(int cntIdx=0; cntIdx < contact->count; cntIdx++) {
                ft_linkCOM.force += contact->wrench->body2Force;
                ft_linkCOM.torque += contact->wrench->body2Torque;
            }
        }
    }

    // Transform the force/torque from the link com to the link origin
    ignition::math::Pose3d linkFrame_H_linkCOM =
        ignition::math::Pose3d(link->GetInertial().get()->Pose().Pos(), ignition::math::Quaterniond::Identity);

    return applyTransformToForceTorque(linkFrame_H_linkCOM, ft_linkCOM);
}

void GazeboYarpContactForceTorques::onUpdate(const gazebo::common::UpdateInfo& info)
{
    // Get force torques from the contact manager and publish it on the port
    if (info.simTime.Double() - m_pimpl->latestUpdateInstantInSeconds < m_pimpl->publishingPeriodInSeconds) {
        return;
    }


    for(size_t i=0; i < m_pimpl->outputFTPorts.size(); i++ )
    {
        // Get the force torque applied by the enviroment on the link in the link frame
        // Get total applied force on link from contact manager
        GazeboYarpContactForceTorques::ForceTorque ft_link =
            getTotalForceTorqueAppliedOnLink(m_pimpl->outputFTPorts[i].link);


        // Transform the force torque  in the desired frame
        GazeboYarpContactForceTorques::ForceTorque ft_specifiedFrame =
            applyTransformToForceTorque(m_pimpl->outputFTPorts[i].publishingFrame_H_linkFrame, ft_link);

        // Convert the force/torque in YARP format
        //
        toYARP(ft_specifiedFrame, m_pimpl->outputFTPorts[i].output_vector);

        // write on port
        m_pimpl->outputFTPorts[i].output_port->prepare() = m_pimpl->outputFTPorts[i].output_vector;
        m_pimpl->outputFTPorts[i].output_port->write();
    }

    m_pimpl->latestUpdateInstantInSeconds = info.simTime.Double();
    return;
}

void gazebo::GazeboYarpContactForceTorques::onReset()
{
    m_pimpl->latestUpdateInstantInSeconds = -1.0;
}


}

