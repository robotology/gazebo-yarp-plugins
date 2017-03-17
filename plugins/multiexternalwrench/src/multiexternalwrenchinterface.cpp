#include <multiexternalwrenchinterface.h>

using namespace gazebo;

MultiExternalWrenchInterface::MultiExternalWrenchInterface()
{
    this->m_wrenchToApply.force.resize(3,0);
    this->m_wrenchToApply.torque.resize(3,0);
    this->m_wrenchToApply.duration = 0.0;
    timeIni = 0;
    
    m_newCommand = false;

}

MultiExternalWrenchInterface::~MultiExternalWrenchInterface()
{
    //Stop all the server threads
    this->m_updateConnection.reset();
}
