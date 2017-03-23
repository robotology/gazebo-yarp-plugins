#include <externalwrench.h>

int ExternalWrench::count = 0;

//Initializing wrench command
ExternalWrench::ExternalWrench()
{
    yInfo() << "New external wrench initialization";
    count++;
    tick = yarp::os::Time::now();
    duration_done = false;
    wrench = new wrenchCommand();
}

bool ExternalWrench::setWrench(physics::ModelPtr& _model,yarp::os::Bottle& cmd)
{
    model = _model;
    wrench->link_name = cmd.get(0).asString();
    yInfo() << "Link name : " << wrench->link_name;
    getLink();
    
    wrench->force.Set(cmd.get(1).asDouble(),cmd.get(2).asDouble(),cmd.get(3).asDouble());
    std::cout << "Force values : " << wrench->force << std::endl;
    wrench->torque.Set(cmd.get(4).asDouble(),cmd.get(5).asDouble(),cmd.get(6).asDouble());
    std::cout << "Torque values : " << wrench->torque << std::endl;
    wrench->duration = cmd.get(7).asDouble();
    yInfo() << "Wrench duration : " << wrench->duration;
    yInfo() << "Set new wrench values";
}

bool ExternalWrench::getLink()
{
    //yInfo() << "Getting the link in the model";
    //Getting the link from link linkName
    model_links = model->GetLinks();
    for(int i = 0; i < model_links.size(); i++)
    {
        //yInfo() << "Number of link in the model : " << model_links.size();
        std::string candidate_link_name = model_links[i]->GetScopedName();
        //yInfo() << "Candidate link full scoped name : " << candidate_link_name;
        std::size_t lastcolon = candidate_link_name.rfind(":");
        std::string unscoped_link_name =  candidate_link_name.substr(lastcolon+1,std::string::npos);
        //yInfo() << "Candidate link unscoped name : " << unscoped_link_name;
        if(unscoped_link_name == wrench->link_name)
        {
            link = model_links[i];
            //yInfo() << "Found the link : " << link->GetName();
            
            //Wrench Visual
            this->m_node = transport::NodePtr(new gazebo::transport::Node());
            this->m_node->Init(model->GetWorld()->GetName());
            std::string visual_topic_name = "~/" + wrench->link_name + "_wrench_visual_" + boost::lexical_cast<std::string>(count);
            m_visPub = this->m_node->Advertise<msgs::Visual> (visual_topic_name,10);
            
            // Set the visual's name. This should be unique.
            m_visualMsg.set_name ("__CYLINDER_VISUAL__");

            // Set the visual's parent. This visual will be attached to the parent
            m_visualMsg.set_parent_name(model->GetScopedName());

            // Create a cylinder
            msgs::Geometry *geomMsg = m_visualMsg.mutable_geometry();
            geomMsg->set_type(msgs::Geometry::CYLINDER);
            geomMsg->mutable_cylinder()->set_radius(0.01);
            geomMsg->mutable_cylinder()->set_length(.30);

            // Don't cast shadows
            m_visualMsg.set_cast_shadows ( false );
            break;
        }
    }
    if(link->GetName() != wrench->link_name)
    {
        yError() << "MultiExternalWrenchPluging::External Wrench error: could not find the link!";
        return false;
    }
    return true;
}
 
void ExternalWrench::applyWrench()
{
    
    
    
    tock = yarp::os::Time::now();
    //yInfo() << "Elapsed time : " << (tock - tick) << " , Duration : " << wrench->duration; 
    if((tock-tick) < wrench->duration)
    {
        //yInfo() << "Applying external wrench";
        link->AddForce(wrench->force);
        link->AddTorque(wrench->torque);
        math::Vector3 linkCoGPos = link->GetWorldCoGPose().pos;
        math::Vector3 newZ = wrench->force.Normalize();
        math::Vector3 newX = newZ.Cross(math::Vector3::UnitZ);
        math::Vector3 newY = newZ.Cross(newX);
        math::Matrix4 rotation = math::Matrix4 (newX[0],newY[0],newZ[0],0,newX[1],newY[1],newZ[1],0,newX[2],newY[2],newZ[2],0, 0, 0, 0, 1);
        math::Quaternion forceOrientation = rotation.GetRotation();
        math::Pose linkCoGPose (linkCoGPos - rotation*math::Vector3(0,0,.15),forceOrientation);
        tock = yarp::os::Time::now();
        
        #if GAZEBO_MAJOR_VERSION >= 7
          msgs::Set(m_visualMsg.mutable_pose(), linkCoGPose.Ign());
        #else
          msgs::Set(m_visualMsg.mutable_pose(), linkCoGPose);
        #endif
        
        msgs::Set(m_visualMsg.mutable_material()->mutable_ambient(),common::Color(1,0,0,0.3));
        m_visualMsg.set_visible(1);
        m_visPub->Publish(m_visualMsg);
    }
    else
    {
        m_visualMsg.set_visible(0);
        m_visPub->Publish(m_visualMsg);
        duration_done = true;
    }
}

ExternalWrench::~ExternalWrench()
{
    //delete wrench;
    count--;
}



