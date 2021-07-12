/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia. RBCS Department.
 * Author: Jorhabib Eljaik
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#include "ApplyExternalWrench.hh"
#include <gazebo/physics/PhysicsTypes.hh>

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

    // Get gazebo simulation update period
    gazebo::physics::PhysicsEnginePtr physicsEngine = _model->GetWorld()->Physics();
    m_rpcThread.m_simulationUpdatePeriod = physicsEngine->GetUpdatePeriod();

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->m_updateConnection = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &ApplyExternalWrench::onUpdate, this, _1 ) );

    // Listen to gazebo world reset event
    this->m_resetConnection = event::Events::ConnectWorldReset ( boost::bind ( &ApplyExternalWrench::onReset, this ) );
}

void ApplyExternalWrench::onUpdate(const gazebo::common::UpdateInfo& _info)
{
    // Update last time stamp
    double time = _info.simTime.Double();
    m_rpcThread.setLastTimeStamp(time);

    this->m_lock.lock();
    for(int i = 0; i < m_rpcThread.wrenchesVector.size() ; i++)
    {
        // Update wrench tock time
        yarp::os::Stamp tockTimeStamp = m_rpcThread.getLastTimeStamp();
        double tockTime = tockTimeStamp.getTime();
        m_rpcThread.wrenchesVector.at(i).setTock(tockTime);

        bool duration_check = m_rpcThread.wrenchesVector.at(i).duration_done;
        if(duration_check == false)
        {
            m_rpcThread.wrenchesVector.at(i).applyWrench();
        }
    }
    this->m_lock.unlock();
}

void ApplyExternalWrench::onReset()
{
    this->m_lock.lock();

    // Delete all the wrenches
    if (this->m_rpcThread.wrenchesVector.size() != 0) {
        for (int i = 0; i < this->m_rpcThread.wrenchesVector.size(); i++)
        {
            ExternalWrench wrench = this->m_rpcThread.wrenchesVector.at(i);
            wrench.deleteWrench();
        }
        this->m_rpcThread.wrenchesVector.clear();
    }

    // Change the operation mode to default option 'single'
    this->m_rpcThread.m_mode = "single";

    // Change the wrench smoothing option to default false
    this->m_rpcThread.m_wrenchSmoothing = false;

    // Reset wrench count
    this->m_rpcThread.wrenchCount = 0;

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

    // Set the default smoothing option
    this->m_wrenchSmoothing = false;

    // Set wrench count default value
    this->wrenchCount = 0;

    return true;
}

void RPCServerThread::run()
{
    while ( !isStopping() ) {
        yarp::os::Bottle command;
        m_rpcPort.read ( command,true );
        if ( command.get ( 0 ).asString() == "help" ) {
            this->m_reply.addVocab32( "many" );
            this->m_reply.addString ( "The default operation mode is with single wrench without wrench smoothing" );
            this->m_reply.addString ( "Insert [single] or [multiple] to change the operation mode" );
            this->m_reply.addString ( "Insert [smoothing on] or [smoothing off] to set the wrench smoothing option" );
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
                && ( command.get ( 1 ).isFloat64() || command.get ( 1 ).isInt32() )  && ( command.get ( 2 ).isFloat64() || command.get ( 2 ).isInt32() ) && ( command.get ( 3 ).isFloat64() || command.get ( 3 ).isInt32() ) \
                && ( command.get ( 4 ).isFloat64() || command.get ( 4 ).isInt32() )  && ( command.get ( 5 ).isFloat64() || command.get ( 5 ).isInt32() ) && ( command.get ( 6 ).isFloat64() || command.get ( 6 ).isInt32() )  \
                && ( command.get ( 7 ).isFloat64() || command.get ( 7 ).isInt32() )) ) {
                this->m_message = "[ACK] Correct command format";
                m_lock.lock();
                // new-command flag
                command.addInt32(1);
                m_cmd = command;
                m_lock.unlock();

                if (this->m_mode == "single") {

                    this->m_lock.lock();

                    // Reset wrench count
                    wrenchCount = 0;

                    // Delete the previous wrenches
                    if (wrenchesVector.size() != 0) {
                        for (int i = 0; i < wrenchesVector.size(); i++)
                        {
                            ExternalWrench wrench = wrenchesVector.at(i);
                            wrench.deleteWrench();
                        }
                        wrenchesVector.clear();
                    }

                    this->m_lock.unlock();
                }

                if (command.get(7).asFloat64() > 0 || command.get(7).asInt32() > 0 ) {

                    if (this->m_wrenchSmoothing == true && (command.get(7).asFloat64() <= m_simulationUpdatePeriod)) {
                        this->m_message = this->m_message + " but the entered duration is less than or equal to the simulation update period";
                        this->m_reply.addString ( m_message );
                        this->m_rpcPort.reply ( m_reply );

                        m_reply.clear();
                        command.clear();

                        continue;
                    }

                    // Create new instances of external wrenches
                    ExternalWrench newWrench;
                    if(newWrench.setWrench(m_robotModel, m_cmd, m_simulationUpdatePeriod, m_wrenchSmoothing))
                    {
                        // Update wrench count
                        wrenchCount++;

                        // Set wrench tick time
                        yarp::os::Stamp tickTimeStamp = this->getLastTimeStamp();
                        double tickTime = tickTimeStamp.getTime();
                        newWrench.setTick(tickTime);

                        // Set wrench index
                        newWrench.setWrenchIndex(wrenchCount);

                        // Set wrench color
                        newWrench.setWrenchColor();

                        // Set wrench visual
                        newWrench.setVisual();

                        this->m_message = this->m_message + " and " + command.get(0).asString() + " link found in the model" ;
                        this->m_reply.addString ( m_message);
                        this->m_rpcPort.reply ( m_reply );
                        wrenchesVector.push_back(newWrench);
                    }
                    else
                    {   this->m_message = this->m_message + " but " + command.get(0).asString() + " link not found in the model" ;
                        this->m_reply.addString ( m_message );
                        this->m_rpcPort.reply ( m_reply );
                    }
                }
                else {
                    this->m_message = this->m_message + " but the entered duration is invalid";
                    this->m_reply.addString ( m_message );
                    this->m_rpcPort.reply ( m_reply );
                }
            }
            else if (command.size() == 1 && command.get(0).isString() && (command.get(0).asString() == "single" || command.get(0).asString() == "multiple")) {

                this->m_mode = command.get(0).asString();

                this->m_message = command.get(0).asString() + " wrench operation mode set";

                if (this->m_wrenchSmoothing == true) {
                    this->m_message = this->m_message + " with wrench smoothing on";
                }
                else {
                    this->m_message = this->m_message + " with wrench smoothing off";
                }

                // Reset wrench count
                wrenchCount = 0;

                this->m_lock.lock();

                // Delete the previous wrenches
                if (wrenchesVector.size() != 0) {
                    this->m_message = this->m_message + ". Clearing previous wrenches.";
                    for (int i = 0; i < wrenchesVector.size(); i++)
                    {
                        ExternalWrench wrench = wrenchesVector.at(i);
                        wrench.deleteWrench();
                    }
                    wrenchesVector.clear();
                }

                this->m_lock.unlock();

                this->m_reply.addString (m_message);
                this->m_rpcPort.reply ( m_reply );

            }
            else if (command.size() == 2 && command.get(0).isString() && command.get(1).isString() && \
                    (command.get(0).asString() == "smoothing" && (command.get(1).asString() == "on" || command.get(1).asString() == "off"))) {

                if (command.get(1).asString() == "on") {
                    this->m_wrenchSmoothing = true;
                    this->m_message = "Wrench smoothing is on starting from the next wrench";
                }
                else if (command.get(1).asString() == "off") {
                    this->m_wrenchSmoothing = false;
                    this->m_message = "Wrench smoothing is off starting from the next wrench";
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

void RPCServerThread::setLastTimeStamp(double& time)
{
    this->m_lock.lock();

    m_lastTimestamp.update(time);

    this->m_lock.unlock();
}

yarp::os::Stamp RPCServerThread::getLastTimeStamp()
{
    boost::lock_guard<boost::mutex> lock{this->m_lock};

    return m_lastTimestamp;
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
