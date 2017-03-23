#ifndef APPLYMULTIEXTERNALWRENCH_H
#define APPLYMULTIEXTERNALWRENCH_H

#include <externalwrench.h>

class RPCServerThread: public yarp::os::Thread
{
private:
    
    ExternalWrench *newWrench;
    
    yarp::os::RpcServer m_rpcPort;
    yarp::os::Bottle m_cmd;
    yarp::os::Bottle m_reply;
    
    std::string m_robotName;
    physics::ModelPtr m_robotModel;
    
public:
    boost::mutex m_lock;
    
    std::unique_ptr<std::vector<ExternalWrench*>> wrenchesVectorPtr{new std::vector<ExternalWrench*>};
    
    virtual bool        threadInit();
    virtual void        run();
    virtual void        threadRelease();
    void                setRobotName(std::string robotName);
    void setRobotModel(physics::ModelPtr robotModel);
    
    yarp::os::Bottle    getCmd();
};

namespace gazebo
{
    class ApplyMultiExternalWrench: public ModelPlugin
    {
    public:
        ApplyMultiExternalWrench();
        ~ApplyMultiExternalWrench();
        void applyWrenchs();
        void Load (physics::ModelPtr _model, sdf::ElementPtr _sdf);
   
    private:
        yarp::os::Network m_yarpNet;
        RPCServerThread m_rpcThread;
        yarp::os::Property m_iniParams;
        
        physics::ModelPtr m_myModel;
        std::string m_modelScope;
        std::string robotName;
        
        boost::mutex m_lock;
        event::ConnectionPtr m_updateConnection;
        
    };
}



#endif