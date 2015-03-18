#ifndef SHOWMODELCOM_HH
#define SHOWMODELCOM_HH

#include <iostream>
#include <string>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <yarp/os/Network.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Vocab.h>

namespace gazebo
{
class ShowModelCoM : public ModelPlugin
{
private:
  transport::NodePtr m_node;

public:
    ShowModelCoM();
    virtual ~ShowModelCoM();
    std::string retrieveSubscope(gazebo::physics::Link_V& v, std::string  scope);

    /// \brief Robot name that will be used to open rpc port
    std::string          robotName;
    double               timeIni;

protected:
    // Inherited
    void Load ( physics::ModelPtr _model, sdf::ElementPtr _sdf );
    // Inherited
    virtual void UpdateChild();


private:
    yarp::os::Network       m_yarpNet;

    physics::ModelPtr       m_myModel;

    std::string             m_modelScope;
    event::ConnectionPtr    m_updateConnection;

    transport::PublisherPtr m_visPub;
    msgs::Visual            m_visualMsg;

    yarp::os::Port          m_com_port;

};

}



#endif
