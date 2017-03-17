#include <externalwrenchserver.h>

void ExternalWrenchServer::setLinkName(std::string& link)
{
    this->m_linkName = link;
}

void ExternalWrenchServer::setDurationBuffer(double d)
{
    this->m_durationBiffer = d;
}

double ExternalWrenchServer::getDurationBuffer()
{
    return this->m_durationBiffer;
}

bool ExternalWrenchServer::threadInit()
{
    //Setting the link name and command details here
    
    m_cmd.addDouble ( 0 ); // Force  coord. x
    m_cmd.addDouble ( 0 ); // Force  coord. y
    m_cmd.addDouble ( 0 ); // Force  coord. z
    m_cmd.addDouble ( 0 ); // Torque coord. x
    m_cmd.addDouble ( 0 ); // Torque coord. y
    m_cmd.addDouble ( 0 ); // Torque coord. z
    m_cmd.addDouble ( 0 ); // Wrench duration
}

void ExternalWrenchServer::onStop()
{
yarp::os::Thread::onStop();
}

void ExternalWrenchServer::run()
{

}

void ExternalWrenchServer::threadRelease()
{
    //This should be called after duration
    yarp::os::Thread::threadRelease();
}


