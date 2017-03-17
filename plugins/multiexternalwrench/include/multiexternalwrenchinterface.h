#ifndef YARPGAZEBO_MULTIEXTERNALWRENCHINTERFACE_H
#define YARPGAZEBO_MULTIEXTERNALWRENCHINTERFACE_H

#include <externalwrenchserver.h>

namespace gazebo
{
    class MultiExternalWrenchInterface : public ModelPlugin
    {
    public:    
        struct wrench
        {
            yarp::sig::Vector force;
            yarp::sig::Vector torque;
            double duration;
        };
    private:
        
        std::string robot_name;
        std::string link_name;
        double timeIni;
        
        yarp::os::RpcServer *m_rpcport;
        yarp::os::Network *m_network;
        yarp::os::Property *m_parameters;
        
        physics::ModelPtr m_model;
        physics::LinkPtr m_link;
        
        wrench m_wrenchToApply;
        boost::mutex m_lock;
        transport::NodePtr m_node;
        event::ConnectionPtr m_updateConnection;
        transport::PublisherPtr m_visPub;
        bool m_newCommand;
        
    public:
                
        MultiExternalWrenchInterface();
        ~MultiExternalWrenchInterface();
        
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
        virtual void updateChild();
        
    };
    
    GZ_REGISTER_MODEL_PLUGIN(MultiExternalWrenchInterface);
}

#endif