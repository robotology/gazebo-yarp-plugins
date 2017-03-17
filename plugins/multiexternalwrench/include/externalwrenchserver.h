#ifndef EXTERNALWRENCHSERVER_H
#define EXTERNALWRENCHSERVER_H

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

class ExternalWrenchServer: public yarp::os::Thread
{   
private:
    yarp::os::Bottle m_cmd;
    yarp::os::Bottle m_reply;
    
    boost::mutex m_lock;
    std::string m_robotName;
    std::string m_linkName;
    double m_durationBiffer;
    
public:
    virtual bool threadInit();
    virtual void run();
    virtual void threadRelease();
    
    void setLinkName(std::string&);
    void setDurationBuffer(double d);
    double getDurationBuffer();
    
    virtual void onStop();
};

#endif
