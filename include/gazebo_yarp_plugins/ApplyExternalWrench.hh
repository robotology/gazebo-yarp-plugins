#ifndef __APPLY_EXTERNAL_WRENCH_HH__
#define __APPLY_EXTERNAL_WRENCH_HH__

#include <iostream>
#include <string>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <yarp/os/Network.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>


// A YARP Thread class that will be used to read the rpc port to apply external wrench
class RPCServerThread: public yarp::os::Thread
{
public:
    virtual bool        threadInit();
    virtual void        run();
    virtual void        threadRelease();
    yarp::os::Bottle    get_cmd();
private:
    yarp::os::RpcServer _rpcPort;
    yarp::os::Bottle    _cmd;
    yarp::os::Bottle    _reply;
    /// \brief Mutex to lock reading and writing of _cmd
    boost::mutex        _lock;

};


namespace gazebo
{
class ApplyExternalForce : public ModelPlugin
{
public:
    ApplyExternalForce();
    virtual ~ApplyExternalForce();

    struct wrench {
        yarp::sig::Vector force;
        yarp::sig::Vector torque;
    };

protected:
    // Inherited
    void Load ( physics::ModelPtr _model, sdf::ElementPtr _sdf );
    // Inherited
    virtual void UpdateChild();

private:
    yarp::os::Network    _yarpNet;
    RPCServerThread      _rpcThread;

    physics::ModelPtr    _myModel;
    /// \brief Link on which the wrench will be applied
    std::string          _link_name;
    /// \brief Link the plugin is attached to
    physics::LinkPtr     _onLink;
    /// \brief Wrench to be applied on the body
    wrench		 _wrench_to_apply;

    /// \brief Mutex to lock access
    boost::mutex         _lock;

    /// \brief Pointer to the update event connection
    event::ConnectionPtr _update_connection;
};

}



#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
