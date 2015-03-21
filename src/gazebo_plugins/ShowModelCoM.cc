#include "ShowModelCoM.hh"

namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN ( ShowModelCoM )

ShowModelCoM::ShowModelCoM()
{

}

ShowModelCoM::~ShowModelCoM()
{
    printf ( "*** GazeboYarpShowModelCoM closing ***\n" );
    this->m_com_port.close();
    gazebo::event::Events::DisconnectWorldUpdateBegin ( this->m_updateConnection );
    printf ( "Goodbye from ShowModelCoM plugin\n" );
}

void ShowModelCoM::UpdateChild()
{
    gazebo::physics::Link_V links = m_myModel->GetLinks();

    double mass_acc = 0.0;
    gazebo::math::Vector3 weighted_position_acc = gazebo::math::Pose::Zero.pos;
    for(unsigned int i = 0; i < links.size(); ++i)
    {
        gazebo::physics::LinkPtr link = links[i];
        gazebo::math::Vector3 wordlCoG_i = link->GetWorldCoGPose().pos;
        double mass_i = link->GetInertial()->GetMass();

        weighted_position_acc += mass_i*wordlCoG_i;
        mass_acc += mass_i;
    }


    gazebo::math::Vector3 wordlCoGModel = weighted_position_acc/mass_acc;

    yarp::os::Bottle bot;
    bot.addDouble(wordlCoGModel.x);
    bot.addDouble(wordlCoGModel.y);
    bot.addDouble(wordlCoGModel.z);
    this->m_com_port.write(bot);

    gazebo::math::Pose WorldCoGPose = gazebo::math::Pose::Zero;
    WorldCoGPose.pos = wordlCoGModel;

    msgs::Set ( m_visualMsg.mutable_pose(), WorldCoGPose );
    msgs::Set ( m_visualMsg.mutable_material()->mutable_ambient(),common::Color ( 1,0,0,0.3 ) );
    m_visualMsg.set_visible ( 1 );
    m_visPub->Publish ( m_visualMsg );

}

void ShowModelCoM::Load ( physics::ModelPtr _model, sdf::ElementPtr _sdf )
{
    // Check if yarp network is active;
    if ( !this->m_yarpNet.checkNetwork() ) {
        printf ( "ERROR Yarp Network was not found active in ApplyExternalWrench plugin\n" );
        return;
    }

    // What is the parent name??
    this->m_modelScope = _model->GetScopedName();
    printf ( "Scoped name: %s\n",this->m_modelScope.c_str() );

    std::string port_name = "/" + this->m_modelScope + "/CoMInWorld:o";
    this->m_com_port.open(port_name);

    // Copy the pointer to the model to access later from UpdateChild
    this->m_myModel = _model;

    bool configuration_loaded = false;

    // Read robot name
    std::cout << "ShowModelCoM: robot name from sdf description will be used!"<<std::endl;
    this->robotName = _model->GetName();
    printf ( "ShowModelCoM: robotName is %s \n",robotName.c_str() );
    configuration_loaded = true;

    /// ############ TRYING TO MODIFY VISUALS #######################################################


    this->m_node = transport::NodePtr ( new gazebo::transport::Node() );

    this->m_node->Init ( _model->GetWorld()->GetName() );
    m_visPub = this->m_node->Advertise<msgs::Visual> ( "~/visual", 10 );

    // Set the visual's name. This should be unique.
    m_visualMsg.set_name ( "__COM_VISUAL__" );

    // Set the visual's parent. This visual will be attached to the parent
    m_visualMsg.set_parent_name ( _model->GetScopedName() );

    // Create a cylinder
    msgs::Geometry *geomMsg = m_visualMsg.mutable_geometry();
    geomMsg->set_type ( msgs::Geometry::SPHERE );
    geomMsg->mutable_sphere()->set_radius ( 0.02 );

    // Don't cast shadows
    m_visualMsg.set_cast_shadows ( false );

    /// ############ END: TRYING TO MODIFY VISUALS #####################################################

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->m_updateConnection = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &ShowModelCoM::UpdateChild, this ) );
}

}




