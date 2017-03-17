#ifndef YARPGAZEBO_MULTIEXTERNALWRENCHINTERFACE_H
#define YARPGAZEBO_MULTIEXTERNALWRENCHINTERFACE_H

#include <externalwrenchserver.h>

namespace gazebo
{
    class MultiExternalWrenchInterface : public ModelPlugin
    {
    public:    
   
    private:

        yarp::os::RpcServer *m_rpcport;
        yarp::os::Network *m_network;
        yarp::os::Property *m_parameters;
        
        gazebo::ExternalWrenchServer m_mew_server;
        
    public:
                
        MultiExternalWrenchInterface();
        ~MultiExternalWrenchInterface();
        
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
    };
    
    GZ_REGISTER_MODEL_PLUGIN(MultiExternalWrenchInterface);
}

#endif