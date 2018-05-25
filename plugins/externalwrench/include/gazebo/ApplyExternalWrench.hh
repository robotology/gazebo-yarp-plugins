#ifndef APPLYEXTERNALWRENCH_HH
#define APPLYEXTERNALWRENCH_HH

#include <iostream>
#include <string>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>

#include <yarp/os/Network.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Vocab.h>


// A YARP Thread class that will be used to read the rpc port to apply external wrench
class RPCServerThread: public yarp::os::Thread
{
public:
    virtual bool        threadInit();
    virtual void        run();
    virtual void        threadRelease();
    yarp::os::Bottle    getCmd();
    void                setRobotName(std::string robotName);
    void                setScopedName(std::string scopedName);
    void                setDefaultLink(const std::string& defaultLink);
    void                setDurationBuffer(double d);
    double              getDurationBuffer();
    void                setNewCommandFlag(std::int32_t flag);
    virtual void        onStop();
private:
    yarp::os::RpcServer m_rpcPort;
    yarp::os::Bottle    m_cmd;
    yarp::os::Bottle    m_reply;
    /// \brief Mutex to lock reading and writing of _cmd
    boost::mutex        m_lock;
    std::string         m_robotName;
    std::string         m_scopedName;
    std::string         m_defaultLink;
    double              m_durationBuffer;
};


namespace gazebo
{
class ApplyExternalWrench : public ModelPlugin
{
private:
  transport::NodePtr m_node;

public:
    ApplyExternalWrench();
    virtual ~ApplyExternalWrench();
    std::string retrieveSubscope(gazebo::physics::Link_V& v, std::string  scope);

    struct wrench {
        yarp::sig::Vector force;
        yarp::sig::Vector torque;
        double            duration;
    };

    /// \brief Robot name that will be used to open rpc port
    std::string          robotName;
    double               timeIni;
//     yarp::os::Bottle     bufferBottle;


protected:
    // Inherited
    void Load ( physics::ModelPtr _model, sdf::ElementPtr _sdf );
    // Inherited
    virtual void UpdateChild();


private:
    yarp::os::Network       m_yarpNet;
    RPCServerThread         m_rpcThread;
    yarp::os::Property      m_iniParams;

    physics::ModelPtr       m_myModel;
    /// \brief Link on which the wrench will be applied
    std::string             m_modelScope;
    std::string             m_subscope;
    std::string             m_linkName;
    /// \brief Link the plugin is attached to
    physics::LinkPtr        m_onLink;
    /// \brief Wrench to be applied on the body
    wrench                  m_wrenchToApply;

    /// \brief Mutex to lock access
    boost::mutex            m_lock;

    /// \brief Pointer to the update event connection
    event::ConnectionPtr    m_updateConnection;

    transport::PublisherPtr m_visPub;
    msgs::Visual            m_visualMsg;

    bool                    m_newCommand;

};

}



#endif
