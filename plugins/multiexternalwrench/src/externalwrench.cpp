#include <externalwrench.h>

//Initializing wrench command
bool ExternalWrench::threadInit()
{
    yInfo() << "Thread initialization";
    wrench = new wrenchCommand();
    return true;
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
    yInfo() << "Set new wrench values done";
}

bool ExternalWrench::getLink()
{
    yInfo() << "Getting the link in the model";
    //Getting the link from link linkName
    model_links = model->GetLinks();
    for(int i = 0; i < model_links.size(); i++)
    {
        std::string candidate_link_name = model_links[i]->GetScopedName();
        yInfo() << "Candidate link name : " << candidate_link_name;
        std::size_t lastcolon = candidate_link_name.rfind(":");
        std::string unscoped_link_name =  candidate_link_name.substr(lastcolon+1,std::string::npos);
        if(unscoped_link_name == wrench->link_name)
        {
            link = model_links[i];
            yInfo() << "Found the link : " << link->GetName();
            break;
        }
        else{
            yError() << "MultiExternalWrenchInterface error: could not find the link!";
            return false;
        }
    }
}
 
void ExternalWrench::run()
{
    updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ExternalWrench::applyWrench,this));
}
void ExternalWrench::applyWrench()
{
    tick = yarp::os::Time::now();
    tock = yarp::os::Time::now();
    if((tock-tick) < wrench->duration)
    {
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
    }
    else this->threadRelease();
}

void ExternalWrench::threadRelease()
{
    delete wrench;
    yarp::os::Thread::threadRelease();
}

