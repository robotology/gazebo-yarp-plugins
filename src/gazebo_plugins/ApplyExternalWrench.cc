#include "gazebo_yarp_plugins/ApplyExternalWrench.hh"

namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN ( ApplyExternalWrench )

ApplyExternalWrench::ApplyExternalWrench()
{
    this->_wrench_to_apply.force.resize ( 3,0 );
    this->_wrench_to_apply.torque.resize ( 3,0 );
    this->_duration = 0;
    time_ini = 0;
    this->_timeChanged = false;
    // Default link on which wrenches are applied
//     _link_name="neck_1";
}
ApplyExternalWrench::~ApplyExternalWrench()
{
    _rpcThread.stop();
}

void ApplyExternalWrench::UpdateChild()
{
    // Reading apply wrench command
    yarp::os::Bottle tmpBottle;

    // Copying command
    this->_lock.lock();
    tmpBottle = this->_rpcThread.get_cmd();
    this->_lock.unlock();

    // initialCmdBottle will contain the initial bottle in RPCServerThread
    static yarp::os::Bottle prevCmdBottle = tmpBottle;

    // Parsing command
    this->_link_name = tmpBottle.get ( 0 ).asString();
    for ( int i=1; i<4; i++ ) {
        _wrench_to_apply.force[i-1] = tmpBottle.get ( i ).asDouble();
    }
    for ( int i=4; i<7; i++ ) {
        _wrench_to_apply.torque[i-4] = tmpBottle.get ( i ).asDouble();
    }

    this->_onLink  = _myModel->GetLink ( std::string ( this->_modelScope + "::" + this->_link_name ) );
    if ( !this->_onLink ) {
        std::cout<<"ERROR ApplyWrench plugin: link named "<< this->_link_name<< "not found"<<std::endl;
        return;
    }

    // Get wrench duration
    this->_duration = tmpBottle.get ( 7 ).asDouble();

    // Copying command to force and torque Vector3 variables
    math::Vector3 force ( this->_wrench_to_apply.force[0], this->_wrench_to_apply.force[1], this->_wrench_to_apply.force[2] );
    math::Vector3 torque ( this->_wrench_to_apply.torque[0], this->_wrench_to_apply.torque[1], this->_wrench_to_apply.torque[2] );
//     printf ( "Applying wrench:( Force: %s ) on link %s \n",_wrench_to_apply.force.toString().c_str(), _link_name.c_str() );
    // Applying wrench to the specified link. If durationBuffer in rpcThread is different from duration in ApplyExternalWrench it means that duration has not been updated via rpc


    // Taking duration of the applied force into account
    static bool applying_force_flag = 0;
    // If the rpc command changed update initial time to check duration
    if ( prevCmdBottle != tmpBottle ) {
        time_ini = yarp::os::Time::now();
        prevCmdBottle = tmpBottle;
        applying_force_flag = 1;
    }

    double time_current = yarp::os::Time::now();
    // This has to be done during the specified duration
    if ( applying_force_flag && ( time_current - time_ini ) < this->_duration ) {
        printf ( "Applying external force on the robot for %f seconds...\n", this->_duration );
        this->_onLink->AddForce ( force );
        this->_onLink->AddTorque ( torque );
        math::Vector3 linkCoGPos = this->_onLink->GetWorldCoGPose().pos; // Get link's COG position where wrench will be applied
        math::Vector3 newZ = force.Normalize(); // normalized force. I want the z axis of the cylinder's reference frame to coincide with my force vector
        math::Vector3 newX = newZ.Cross ( math::Vector3::UnitZ );
        math::Vector3 newY = newZ.Cross ( newX );
        math::Matrix4 rotation = math::Matrix4 ( newX[0],newY[0],newZ[0],0,newX[1],newY[1],newZ[1],0,newX[2],newY[2],newZ[2],0, 0, 0, 0, 1 );
        math::Quaternion forceOrientation = rotation.GetRotation();
        math::Pose linkCoGPose ( linkCoGPos - rotation*math::Vector3 ( 0,0,.15 ), forceOrientation );
        msgs::Set ( _visualMsg.mutable_pose(), linkCoGPose );
        msgs::Set ( _visualMsg.mutable_material()->mutable_ambient(),common::Color ( 1,0,0,0.3 ) );
        _visualMsg.set_visible ( 1 );
        _visPub->Publish ( _visualMsg );
    }

    if ( applying_force_flag && ( time_current - time_ini ) > this->_duration ) {
        printf ( "applying_force_flag set to zero because duration has been met\n" );
        applying_force_flag = 0;
        _visualMsg.set_visible ( 0 );
        _visPub->Publish ( _visualMsg );
    }

}

void ApplyExternalWrench::Load ( physics::ModelPtr _model, sdf::ElementPtr _sdf )
{
    // Check if yarp network is active;
    if ( !this->_yarpNet.checkNetwork() ) {
        printf ( "ERROR Yarp Network was not found active in ApplyExternalWrench plugin" );
        return;
    }

    // What is the parent name??
    this->_modelScope = _model->GetScopedName();
    printf ( "Scoped name: %s\n",this->_modelScope.c_str() );

    // Copy the pointer to the model to access later from UpdateChild
    this->_myModel = _model;

    bool configuration_loaded = false;

    // Read robot name
    if ( _sdf->HasElement ( "robotNamefromConfigFile" ) ) {
        std::string ini_robot_name      = _sdf->Get<std::string> ( "robotNamefromConfigFile" );
        std::string ini_robot_name_path =  gazebo::common::SystemPaths::Instance()->FindFileURI ( ini_robot_name );

        if ( ini_robot_name_path != "" && this->_iniParams.fromConfigFile ( ini_robot_name_path.c_str() ) ) {
            std::cout << "ApplyExternalWrench: Found robotNamefromConfigFile in "<< ini_robot_name_path << std::endl;
            yarp::os::Value robotNameParam = _iniParams.find ( "gazeboYarpPluginsRobotName" );
            this->robot_name = robotNameParam.asString();
            printf ( "ApplyExternalWrench: robotName is %s \n",robot_name.c_str() );
            _rpcThread.setRobotName ( robot_name );
            _rpcThread.setScopedName ( this->_modelScope );
            configuration_loaded = true;
        } else {
            printf ( "ERROR trying to get robot configuration file\n" );
            return;
        }
    } else {
        printf ( "ERROR: SDF file does not contain a <robotNamefromConfigFile> tag. A robot name is needed\n" );
        return;
    }

    // Starting RPC thread to read desired wrench to be applied
    if ( !_rpcThread.start() ) {
        printf ( "ERROR: rpcThread did not start correctly\n" );
    }


    /// ############ TRYING TO MODIFY VISUALS #######################################################


    this->node = transport::NodePtr ( new gazebo::transport::Node() );

    this->node->Init ( _model->GetWorld()->GetName() );
    _visPub = this->node->Advertise<msgs::Visual> ( "~/visual", 10 );

    // Set the visual's name. This should be unique.
    _visualMsg.set_name ( "__CYLINDER_VISUAL__" );

    // Set the visual's parent. This visual will be attached to the parent
    _visualMsg.set_parent_name ( _model->GetScopedName() );

    // Create a cylinder
    msgs::Geometry *geomMsg = _visualMsg.mutable_geometry();
    geomMsg->set_type ( msgs::Geometry::CYLINDER );
    geomMsg->mutable_cylinder()->set_radius ( 0.01 );
    geomMsg->mutable_cylinder()->set_length ( .30 );

    // Don't cast shadows
    _visualMsg.set_cast_shadows ( false );

    /// ############ END: TRYING TO MODIFY VISUALS #####################################################


    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->_update_connection = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &ApplyExternalWrench::UpdateChild, this ) );
}

}


// ############ RPCServerThread class ###############

void RPCServerThread::setRobotName ( std::string robotName )
{
    this->_robotName = robotName;
}

void RPCServerThread::setScopedName ( std::string scopedName )
{
    this->_scopedName = scopedName;
}

void RPCServerThread::setDurationBuffer ( double d )
{
    this->_durationBuffer = d;
}

double RPCServerThread::getDurationBuffer()
{
    return this->_durationBuffer;
}



bool RPCServerThread::threadInit()
{

    printf ( "Starting RPCServerThread\n" );
    printf ( "Opening rpc port\n" );
    if ( !_rpcPort.open ( std::string ( "/"+_robotName + "/applyExternalWrench/rpc:i" ).c_str() ) ) {
        printf ( "ERROR opening RPC port /applyExternalWrench\n" );
        return false;
    }
    // Default link on which wrenches are applied
    _cmd.addString ( this->_scopedName + "::l_arm" );
    _cmd.addDouble ( 0 ); // Force  coord. x
    _cmd.addDouble ( 0 ); // Force  coord. y
    _cmd.addDouble ( 0 ); // Force  coord. z
    _cmd.addDouble ( 0 ); // Torque coord. x
    _cmd.addDouble ( 0 ); // Torque coord. y
    _cmd.addDouble ( 0 ); // Torque coord. z
    _cmd.addDouble ( 0 ); // Wrench duration

    this->_durationBuffer = _cmd.get ( 7 ).asDouble();

    return true;
}
void RPCServerThread::run()
{
    while ( !isStopping() ) {
        yarp::os::Bottle command;

        _rpcPort.read ( command,true );
        this->_reply.addString ( "[ACK]" );
        this->_rpcPort.reply ( this->_reply );
        _reply.clear();
        _lock.lock();
        _cmd = command;
        _lock.unlock();
    }
}
void RPCServerThread::threadRelease()
{
    yarp::os::Thread::threadRelease();
    printf ( "Goodbye from RPC thread\n" );
}
yarp::os::Bottle RPCServerThread::get_cmd()
{
    return _cmd;
}
