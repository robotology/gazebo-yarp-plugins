#include <applymultiexternalwrench.h>

namespace gazebo
{
    GZ_REGISTER_MODEL_PLUGIN(ApplyMultiExternalWrench)
    
ApplyMultiExternalWrench::ApplyMultiExternalWrench()
{

}

ApplyMultiExternalWrench::~ApplyMultiExternalWrench()
{
    m_rpcThread.stop();
}

void ApplyMultiExternalWrench::Load ( physics::ModelPtr _model, sdf::ElementPtr _sdf )
{
    if ( !this->m_yarpNet.checkNetwork() ) {
        yError ( "ERROR Yarp Network was not found active in ApplyExternalWrench plugin" );
        return;
    }

    this->m_modelScope = _model->GetScopedName();

    // Copy the pointer to the model to access later from UpdateChild
    this->m_myModel = _model;

    bool configuration_loaded = false;

    // Read robot name
    if ( _sdf->HasElement ( "robotNamefromConfigFile" ) ) {
        std::string iniRobotName      = _sdf->Get<std::string> ( "robotNamefromConfigFile" );
        std::string iniRobotNamePath =  gazebo::common::SystemPaths::Instance()->FindFileURI ( iniRobotName );

        if ( iniRobotNamePath != "" && this->m_iniParams.fromConfigFile ( iniRobotNamePath.c_str() ) ) {
            yarp::os::Value robotNameParam = m_iniParams.find ( "gazeboYarpPluginsRobotName" );
            this->robotName = robotNameParam.asString();
            
            m_rpcThread.setRobotName  ( robotName );
            m_rpcThread.setRobotModel(_model);
            gazebo::physics::Link_V links = _model->GetLinks();
            std::string defaultLink = links[0]->GetName();
            configuration_loaded = true;
        } else {
            yError ( "ERROR trying to get robot configuration file" );
            return;
        }
    } else {
            this->robotName = _model->GetName();
            m_rpcThread.setRobotName ( robotName );
            gazebo::physics::Link_V links = _model->GetLinks();
            configuration_loaded = true;
    }

    // Starting RPC thread to read desired wrench to be applied
    if ( !m_rpcThread.start() ) {
        yError ( "ERROR: rpcThread did not start correctly" );
    }

}

}

void RPCServerThread::setRobotModel(physics::ModelPtr robotModel)
{
    m_robotModel = robotModel;
}


bool RPCServerThread::threadInit()
{

    if ( !m_rpcPort.open ( std::string ( "/"+m_robotName + "/applyMultiExternalWrench/rpc:i" ).c_str() ) ) {
        yError ( "ERROR opening RPC port /applyExternalWrench" );
        return false;
    }
  
    return true;
}

void RPCServerThread::run()
{
    while(!isStopping())
    {
        yarp::os::Bottle command;
        m_rpcPort.read ( command,true );
        if(command.get(0).asString() == "help")
        {
            this->m_reply.addVocab ( yarp::os::Vocab::encode ( "many" ) );
            this->m_reply.addString ( "Insert a command with the following format:" );
            this->m_reply.addString ( "[link] [force] [torque] [duration]" );
            this->m_reply.addString ( "e.g. chest 10 0 0 0 0 0 1");
            this->m_reply.addString ( "[link]:     (string) Link ID of the robot as specified in robot's SDF" );
            this->m_reply.addString ( "[force]:    (double x, y, z) Force components in N w.r.t. world reference frame" );
            this->m_reply.addString ( "[torque]:   (double x, y, z) Torque components in N.m w.r.t world reference frame" );
            this->m_reply.addString ( "[duration]: (double) Duration of the applied force in seconds" );
            this->m_reply.addString ( "Note: The reference frame is the base/root robot frame with x pointing backwards and z upwards.");
            this->m_rpcPort.reply ( this->m_reply );
        }
        else{
            if(command.get(0).isString() \
                    && ( command.get ( 1 ).isDouble() || command.get ( 1 ).isInt() )  && ( command.get ( 2 ).isDouble() || command.get ( 2 ).isInt() ) && ( command.get ( 3 ).isDouble() || command.get ( 3 ).isInt() ) \
                    && ( command.get ( 4 ).isDouble() || command.get ( 4 ).isInt() )  && ( command.get ( 5 ).isDouble() || command.get ( 5 ).isInt() ) && ( command.get ( 6 ).isDouble() || command.get ( 6 ).isInt() )  \
                    && ( command.get ( 7 ).isDouble() || command.get ( 7 ).isInt() ) ) {
                this->m_reply.addString ( "[ACK] Correct command format" );
                this->m_rpcPort.reply ( m_reply );
                m_lock.lock();
                // new-command flag
                command.addInt(1);
                m_cmd = command;
                m_lock.unlock();
               
                yInfo() << "Creating new instances of external wrench";
               //Creating new instances of external wrenches
               newWrench = new ExternalWrench();
               newWrench->start();
               newWrench->setWrench(m_robotModel,m_cmd);
               wrenchThreads.push_back(newWrench);
            
            } else {
                this->m_reply.clear();
                this->m_reply.addString ( "ERROR: Incorrect command format" );
                this->m_rpcPort.reply ( this->m_reply );
            }
        }
        m_reply.clear();
        command.clear();
        
        //Now check for duration done flag from all the wrench threads and removes the ones that are done
        yInfo() << "Number of external wrench threads : " << wrenchThreads.size();
        /*for(int i = 0; i < wrenchThreads.size() ; i++)
        {
            bool duration_check = wrenchThreads.at(i)->duration_done;
            if(duration_check==true)
            {
                yInfo() << "Deleting an external wrench thread";
                delete wrenchThreads.at(i);
            }
            
        }*/
    }
}

void RPCServerThread::setRobotName ( std::string robotName )
{
    this->m_robotName = robotName;
}

void RPCServerThread::threadRelease()
{
    yarp::os::Thread::threadRelease();
    m_rpcPort.close();
}

yarp::os::Bottle RPCServerThread::getCmd()
{
    return m_cmd;
}