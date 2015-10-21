/*
 * Copyright (C) 2013-2015 Istituto Italiano di Tecnologia RBCS & ADVR & iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */
#ifndef SHOWMODELCOM_HH
#define SHOWMODELCOM_HH

#include <iostream>
#include <string>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
    class ShowModelCoM;

    namespace transport {
        class Publisher;
        typedef boost::shared_ptr<Publisher> PublisherPtr;
        class Node;
        typedef boost::shared_ptr<Node> NodePtr;
    }

    namespace Physics {
        class Model;
        typedef boost::shared_ptr<Model> ModelPtr;
    }
    namespace event {
        class Connection;
        typedef boost::shared_ptr<Connection> ConnectionPtr;
    }

    namespace msgs {
        class Visual;
    }
}

namespace yarp {
    namespace os {
        class Bottle;

        template <class T>
        class BufferedPort;
    }
}


class gazebo::ShowModelCoM : public gazebo::ModelPlugin
{
public:
    ShowModelCoM();
    virtual ~ShowModelCoM();

    std::string retrieveSubscope(gazebo::physics::Link_V& v, std::string  scope);

protected:
    // Inherited
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);
    // Inherited
    virtual void UpdateChild();

private:
    gazebo::transport::NodePtr m_node;

    gazebo::physics::ModelPtr       m_myModel;

    std::string             m_modelScope;
    gazebo::event::ConnectionPtr    m_updateConnection;

    gazebo::transport::PublisherPtr m_visPub;
    gazebo::msgs::Visual            m_visualMsg;

    yarp::os::BufferedPort<yarp::os::Bottle> *m_comOutputPort;
    
};

#endif
