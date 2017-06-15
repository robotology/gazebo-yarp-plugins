#include <externalwrench.h>

int ExternalWrench::count = 0;

//Initializing wrench command
ExternalWrench::ExternalWrench()
{
    //yInfo() << "New external wrench initialization";
    model_has_link = false;
    
    //srand(boost::lexical_cast<float>(std::time(NULL)));
    srand(rand()%100);
    color[0] = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    color[1] = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    color[2] = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    color[3] = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    
    count++;
    tick = yarp::os::Time::now();
    duration_done = false;
}

bool ExternalWrench::getLink()
{
    //yInfo() << "Getting the link in the model";
    //Getting the link from link linkName
    model_links = model->GetLinks();
    //yInfo() << "Number of links in the model : " << model_links.size();
    for(int i = 0; i < model_links.size(); i++)
    {
        
        std::string candidate_link_name = model_links[i]->GetScopedName();
        //yInfo() << "Candidate link full scoped name : " << candidate_link_name;
        std::size_t lastcolon = candidate_link_name.rfind(":");
        std::string unscoped_link_name =  candidate_link_name.substr(lastcolon+1,std::string::npos);
        //yInfo() << "Candidate link unscoped name : " << unscoped_link_name;
        if(unscoped_link_name == wrenchPtr->link_name)
        {
            link = model_links[i];
            //yInfo() << "Found the link : " << link->GetName();
            
            //Wrench Visual
            this->m_node = transport::NodePtr(new gazebo::transport::Node());
            this->m_node->Init(model->GetWorld()->GetName());
            m_visPub = this->m_node->Advertise<msgs::Visual>("~/visual",100);
            
            // Set the visual's name. This should be unique.
            std::string visual_name = "__" + wrenchPtr->link_name + "__CYLINDER_VISUAL__" + boost::lexical_cast<std::string>(count);
            m_visualMsg.set_name (visual_name);

            // Set the visual's parent. This visual will be attached to the parent
            m_visualMsg.set_parent_name(model->GetScopedName());

            // Create a cylinder
            msgs::Geometry *geomMsg = m_visualMsg.mutable_geometry();
            geomMsg->set_type(msgs::Geometry::CYLINDER);
            geomMsg->mutable_cylinder()->set_radius(0.0015);
            geomMsg->mutable_cylinder()->set_length(.15);

            // Don't cast shadows
            m_visualMsg.set_cast_shadows(false);
            model_has_link = true;
            break;
        }
    }
    if(!model_has_link)
    {
        yError() << "MultiExternalWrenchPluging::External Wrench error: could not find the link in the model!";
        return false;
    }
    return true;
}

bool ExternalWrench::setWrench(physics::ModelPtr& _model,yarp::os::Bottle& cmd)
{
    model = _model;
    wrenchPtr->link_name = cmd.get(0).asString();
    //yInfo() << "Link name : " << wrenchPtr->link_name;
    //yInfo() << "wrench values : " << cmd.get(1).asDouble() << cmd.get(2).asDouble() << cmd.get(3).asDouble() << cmd.get(4).asDouble() << cmd.get(5).asDouble() << cmd.get(6).asDouble();
    //yInfo() << "wrench duration : " << cmd.get(7).asDouble();
    if(getLink())
    {
        force_ = new gazebo::math::Vector3(cmd.get(1).asDouble(),cmd.get(2).asDouble(),cmd.get(3).asDouble());
        torque_ = new gazebo::math::Vector3(cmd.get(4).asDouble(),cmd.get(5).asDouble(),cmd.get(6).asDouble());
        wrenchPtr->duration = cmd.get(7).asDouble();
        
        //std::cout << "Force values : " << force_ << std::endl;
        //std::cout << "Torque values : " << torque_ << std::endl;
        //yInfo() << "Wrench duration : " << wrenchPtr->duration;
        return true;
    }
    else return false;
    //yInfo() << "Set new wrench values";
}
 
void ExternalWrench::applyWrench()
{
    tock = yarp::os::Time::now();
    //yInfo() << "Elapsed time : " << (tock - tick) << " , Duration : " << wrenchPtr->duration; 
    if((tock-tick) < wrenchPtr->duration)
    {
        //yInfo() << "Applying external wrench";
        wrenchPtr->force = *force_;
        wrenchPtr->torque = *torque_;
        //std::cout << wrenchPtr->force << " , " << wrenchPtr->torque << std::endl;
       
        link->AddForce(wrenchPtr->force);
        link->AddTorque(wrenchPtr->torque);
        
        math::Vector3 linkCoGPos = link->GetWorldCoGPose().pos;
        math::Vector3 newZ = wrenchPtr->force.Normalize();
        math::Vector3 newX = newZ.Cross(math::Vector3::UnitZ);
        math::Vector3 newY = newZ.Cross(newX);
        math::Matrix4 rotation = math::Matrix4 (newX[0],newY[0],newZ[0],0,newX[1],newY[1],newZ[1],0,newX[2],newY[2],newZ[2],0, 0, 0, 0, 1);
        math::Quaternion forceOrientation = rotation.GetRotation();
        math::Pose linkCoGPose (linkCoGPos - rotation*math::Vector3(0,0,0.075),forceOrientation);
        tock = yarp::os::Time::now();
        
        #if GAZEBO_MAJOR_VERSION >= 7
          msgs::Set(m_visualMsg.mutable_pose(), linkCoGPose.Ign());
        #else
          msgs::Set(m_visualMsg.mutable_pose(), linkCoGPose);
        #endif
          
        msgs::Set(m_visualMsg.mutable_material()->mutable_ambient(),common::Color(color[0],color[1],color[2],color[3]));
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
    yInfo() << "ExternalWrench Destructor";
    delete force_;
    delete torque_;
    count--;
}



