#include "gazebo_yarp_plugins/ShowModelCoM.hh"

namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN ( ShowModelCoM )

ShowModelCoM::ShowModelCoM()
{

}

ShowModelCoM::~ShowModelCoM()
{
    printf ( "*** GazeboYarpShowModelCoM closing ***\n" );
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


}

void ShowModelCoM::Load ( physics::ModelPtr _model, sdf::ElementPtr _sdf )
{
    // What is the parent name??
    this->m_modelScope = _model->GetScopedName();
    printf ( "Scoped name: %s\n",this->m_modelScope.c_str() );

    // Copy the pointer to the model to access later from UpdateChild
    this->m_myModel = _model;

    bool configuration_loaded = false;

    // Read robot name
    std::cout << "ShowModelCoM: robot name from sdf description will be used!"<<std::endl;
    this->robotName = _model->GetName();
    printf ( "ShowModelCoM: robotName is %s \n",robotName.c_str() );
    configuration_loaded = true;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->m_updateConnection = event::Events::ConnectWorldUpdateBegin ( boost::bind ( &ShowModelCoM::UpdateChild, this ) );
}

}




