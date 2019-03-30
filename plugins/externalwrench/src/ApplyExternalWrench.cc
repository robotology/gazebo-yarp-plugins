/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia. RBCS Department.
 * Author: Jorhabib Eljaik
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "ApplyExternalWrench.hh"

namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN ( ApplyExternalWrench )

ApplyExternalWrench::ApplyExternalWrench()
{
}

ApplyExternalWrench::~ApplyExternalWrench()
{
    m_rpcThread.stop();
    this->m_updateConnection.reset();
}

void ApplyExternalWrench::Load ( physics::ModelPtr _model, sdf::ElementPtr _sdf )
{
    // Check if yarp network is active
    if ( !this->m_yarpNet.checkNetwork() ) {
        yError ( "ERROR Yarp Network was not found active in ApplyExternalWrench plugin" );
        return;
    }

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
            configuration_loaded = true;
        } else {
            yError ( "ERROR trying to get robot configuration file" );
            return;
        }
    } else {
            this->robotName = _model->GetName();
            m_rpcThread.setRobotName ( robotName );
            m_rpcThread.setRobotModel(_model);
            gazebo::physics::Link_V links = _model->GetLinks();
            configuration_loaded = true;
    }

    // Starting RPC thread to read desired wrench to be applied
    if ( !m_rpcThread.start() ) {
        yError ( "ERROR: rpcThread did not start correctly" );
    }

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->m_updateConnection = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &ApplyExternalWrench::applyWrenches, this ) );
}

void ApplyExternalWrench::applyWrenches()
{
    this->m_lock.lock();
    for(int i = 0; i < m_rpcThread.wrenchesVectorPtr->size() ; i++)
    {
        bool duration_check = m_rpcThread.wrenchesVectorPtr->at(i)->duration_done;
        if(duration_check==false)
        {
            m_rpcThread.wrenchesVectorPtr->at(i)->applyWrench();
        }
    }
    this->m_lock.unlock();
}

}


// ############ RPCServerThread class ###############

bool RPCServerThread::threadInit()
{

    if ( !m_rpcPort.open ( std::string ( "/"+m_robotName + "/applyExternalWrench/rpc:i" ).c_str() ) ) {
        yError ( "ERROR opening RPC port /applyExternalWrench" );
        return false;
    }

    // Set the default operation mode
    this->m_mode = "single";

    return true;
}

void RPCServerThread::run()
{
    while ( !isStopping() ) {
        yarp::os::Bottle command;
        m_rpcPort.read ( command,true );
        if ( command.get ( 0 ).asString() == "help" ) {
            this->m_reply.addVocab ( yarp::os::Vocab::encode ( "many" ) );
            this->m_reply.addString ( "The defaul operation mode is with single wrench" );
            this->m_reply.addString ( "Insert [single] or [multiple] to change the operation mode" );
            this->m_reply.addString ( "Insert a command with the following format:" );
            this->m_reply.addString ( "[link] [force] [torque] [duration]" );
            this->m_reply.addString ( "e.g. chest 10 0 0 0 0 0 1");
            this->m_reply.addString ( "[link]:     (string) Link ID of the robot as specified in robot's SDF" );
            this->m_reply.addString ( "[force]:    (double x, y, z) Force components in N w.r.t. world reference frame" );
            this->m_reply.addString ( "[torque]:   (double x, y, z) Torque components in N.m w.r.t world reference frame" );
            this->m_reply.addString ( "[duration]: (double) Duration of the applied force in seconds" );
            this->m_reply.addString ( "Note: The reference frame is the base/root robot frame with x pointing backwards and z upwards.");
            this->m_rpcPort.reply ( this->m_reply );
        } else{
            if((command.size() == 8) && (command.get(0).isString() \
                && ( command.get ( 1 ).isDouble() || command.get ( 1 ).isInt() )  && ( command.get ( 2 ).isDouble() || command.get ( 2 ).isInt() ) && ( command.get ( 3 ).isDouble() || command.get ( 3 ).isInt() ) \
                && ( command.get ( 4 ).isDouble() || command.get ( 4 ).isInt() )  && ( command.get ( 5 ).isDouble() || command.get ( 5 ).isInt() ) && ( command.get ( 6 ).isDouble() || command.get ( 6 ).isInt() )  \
                && ( command.get ( 7 ).isDouble() || command.get ( 7 ).isInt() )) ) {
                this->m_message = "[ACK] Correct command format";
                m_lock.lock();
                // new-command flag
                command.addInt(1);
                m_cmd = command;
                m_lock.unlock();

                if (this->m_mode == "single") {

                    // Delete the previous wrenches
                    if (wrenchesVectorPtr->size() != 0) {
                        for (int i = 0; i < wrenchesVectorPtr->size(); i++)
                        {
                            boost::shared_ptr<ExternalWrench> wrench = wrenchesVectorPtr->at(i);
                            wrench->deleteWrench();
                        }
                        wrenchesVectorPtr->clear();
                    }

                }

                //Creating new instances of external wrenches
                boost::shared_ptr<ExternalWrench> newWrench(new ExternalWrench);
                if(newWrench->setWrench(m_robotModel, m_cmd))
                {
                    this->m_message = this->m_message + " and " + command.get(0).asString() + " link found in the model" ;
                    this->m_reply.addString ( m_message);
                    this->m_rpcPort.reply ( m_reply );
                    wrenchesVectorPtr->push_back(newWrench);
                }
                else
                {   this->m_message = this->m_message + " but " + command.get(0).asString() + " link found in the model" ;
                    this->m_reply.addString ( m_message );
                    this->m_rpcPort.reply ( m_reply );
                }
            }
            else if (command.size() == 1 && command.get(0).isString()) {

                this->m_mode = command.get(0).asString();

                this->m_message = command.get(0).asString() + " wrench operation mode set";

                // Delete the previous wrenches
                if (wrenchesVectorPtr->size() != 0) {
                    this->m_message = this->m_message + " . Clearing previous wrenches.";
                    for (int i = 0; i < wrenchesVectorPtr->size(); i++)
                    {
                        boost::shared_ptr<ExternalWrench> wrench = wrenchesVectorPtr->at(i);
                        wrench->deleteWrench();
                    }
                    wrenchesVectorPtr->clear();
                }

                this->m_reply.addString (m_message);
                this->m_rpcPort.reply ( m_reply );

            }
            else {
                this->m_reply.clear();
                this->m_message = "ERROR: Incorrect command format. Insert [help] to know the correct command format";
                this->m_reply.addString ( m_message );
                this->m_rpcPort.reply ( this->m_reply );
            }
        }
        m_reply.clear();
        command.clear();
    }
}

void RPCServerThread::setRobotModel(physics::ModelPtr robotModel)
{
    m_robotModel = robotModel;
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

void RPCServerThread::setNewCommandFlag(std::int32_t flag)
{
    m_cmd.get( 8 ) = yarp::os::Value(flag);
}

void RPCServerThread::onStop()
{
    m_rpcPort.interrupt();
}
