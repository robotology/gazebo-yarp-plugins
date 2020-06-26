#ifndef GAZEBO_YARP_PLUGINS_APPLYEXTERNALWRENCH_HH
#define GAZEBO_YARP_PLUGINS_APPLYEXTERNALWRENCH_HH

#include <iostream>
#include <string>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <GazeboYarpPlugins/common.h>

#include <yarp/os/Network.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/LogStream.h>

#include "ExternalWrench.hh"

// A YARP Thread class that will be used to read the rpc port to apply external wrench
class RPCServerThread: public yarp::os::Thread
{
public:
    // variable for operation mode
    // single wrench or multiple wrenches
    std::string         m_mode;

    // wrench smoothing flag
    bool                m_wrenchSmoothing;

    // gazebo simulation update period
    double              m_simulationUpdatePeriod;

    int                 wrenchCount;

    virtual bool        threadInit();
    virtual void        run();
    virtual void        threadRelease();
    yarp::os::Bottle    getCmd();
    void                setRobotName(std::string robotName);
    void                setRobotModel(physics::ModelPtr robotModel);
    void                setNewCommandFlag(std::int32_t flag);
    void                setLastTimeStamp(double& time);
    yarp::os::Stamp     getLastTimeStamp();
    virtual void        onStop();

    std::vector<ExternalWrench> wrenchesVector;

private:
    yarp::os::RpcServer m_rpcPort;
    yarp::os::Bottle    m_cmd;
    yarp::os::Bottle    m_reply;
    /// \brief Mutex to lock reading and writing of _cmd
    boost::mutex        m_lock;
    physics::ModelPtr   m_robotModel;
    std::string         m_robotName;
    double              m_durationBuffer;

    std::string         m_message;

    yarp::os::Stamp     m_lastTimestamp;
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
    void onUpdate(const gazebo::common::UpdateInfo&);
    void onReset();

    bool getCandidateLink(physics::LinkPtr& candidateLink, std::string candidateLinkName);

    /// \brief Robot name that will be used to open rpc port
    std::string          robotName;

protected:
    // Inherited
    void Load ( physics::ModelPtr _model, sdf::ElementPtr _sdf );

private:
    yarp::os::Network       m_yarpNet;
    RPCServerThread         m_rpcThread;
    yarp::os::Property      m_iniParams;

    physics::ModelPtr       m_myModel;

    /// \brief Link the plugin is attached to
    physics::LinkPtr        m_onLink;

    /// \brief Mutex to lock access
    boost::mutex            m_lock;

    /// \brief Pointer to the update event connection
    event::ConnectionPtr    m_updateConnection;

    /// \brief Pointer to WorldReset Gazebo event
    event::ConnectionPtr    m_resetConnection;

    transport::PublisherPtr m_visPub;
    msgs::Visual            m_visualMsg;

    bool                    m_newCommand;
};

}



#endif //GAZEBO_YARP_PLUGINS_APPLYEXTERNALWRENCH_HH
