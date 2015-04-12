/*
 * Copyright (C) 2007-2015 Istituto Italiano di Tecnologia RBCS, ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */



#include "Objects.hh"
#include "ObjectsServerImpl.h"
#include "common.h"

#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/Network.h>

#include <sstream>

GZ_REGISTER_MODEL_PLUGIN(gazebo::GazeboYarpObjects)

namespace gazebo {

GazeboYarpObjects::GazeboYarpObjects() : ModelPlugin(), m_network()
{
}

GazeboYarpObjects::~GazeboYarpObjects()
{
}

void GazeboYarpObjects::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    gazeboYarpObjectsLoad(_parent,_sdf);
}

void GazeboYarpObjects::gazeboYarpObjectsLoad(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    m_model = _parent;
    m_world = _parent->GetWorld();

    m_portName = "/world";

    m_rpcPort = new yarp::os::Port();
    if (!m_rpcPort) {
        std::cerr << "GazeboYarpObjects: Failed to create rpc port." << std::endl;
        cleanup();
        return;
    }

    m_clockServer = new ObjectsServerImpl(*this);
    if (!m_clockServer) {
        std::cerr << "GazeboYarpObjects: Could not create Objects Server." << std::endl;
        cleanup();
        return;
    }

    if (!m_clockServer->yarp().attachAsServer(*m_rpcPort)) {
        std::cerr << "GazeboYarpObjects: Failed to attach Objects Server to RPC port." << std::endl;
        cleanup();
        return;
    }

    if (!m_rpcPort->open(m_portName)) {
        std::cerr << "GazeboYarpObjects: Failed to open rpc port " << (m_portName) << std::endl;
        cleanup();
        return;
    }

    if ( !createHandle() ) {
        std::cerr << "Can not create a handle for grasping objects " << std::endl;
        cleanup();
        return;
    }
    ;
}

void GazeboYarpObjects::cleanup()
{
    if (m_rpcPort) {
        m_rpcPort->close();
        delete m_rpcPort; m_rpcPort = 0;
    }

    if (m_clockServer) {
        delete m_clockServer; m_clockServer = 0;
    }
}

bool GazeboYarpObjects::deleteObject(const std::string& object_name)
{
    if (m_world->GetModel(object_name)!=NULL)
    {
        m_world->RemoveModel(object_name);
        return true;
    }
    else
        return false;
}


bool gazebo::GazeboYarpObjects::createHandle()
{

    sdf::SDF handleSDF;
    std::stringstream ss;
    ss << "<sdf version ='1.4'>\
            <model name ='object_handle'>\
            <pose>0 0 0 0 0 0</pose>\
            <link name ='object_handle_link'>\
              <pose>0 0 0 0 0 0</pose>\
              <collision name='collision'>\
                  <geometry>\
                    <box>\
                      <size>0.00001 0.00001 0.00001</size>\
                    </box>\
                  </geometry>\
               </collision>\
               <inertial>\
                 <mass>0.00001</mass>\
                 <inertia>\
                   <ixx>0.00001</ixx>\
                   <ixy>0</ixy>\
                   <ixz>0</ixz>\
                   <iyy>0.00001</iyy>\
                   <iyz>0</iyz>\
                   <izz>0.00001</izz>\
                 </inertia>\
               </inertial>\
            </link>\
          </model>\
        </sdf>";
    std::string handleSDF_str = ss.str();
    handleSDF.SetFromString(handleSDF_str);
    m_world->InsertModelSDF(handleSDF);

    return true;
}

bool hasEnding (std::string const &fullString, std::string const &ending)
{
    std::cout<<fullString<<" "<<ending<<std::endl;
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

gazebo::physics::LinkPtr getLinkInModel(gazebo::physics::ModelPtr model, std::string link_name)
{
    gazebo::physics::Link_V model_links = model->GetLinks();
    for(int i=0; i < model_links.size(); i++ ) {
        std::string candidate_link = model_links[i]->GetScopedName();
        
//         std::cout<<candidate_link<<std::endl;
        if( hasEnding(candidate_link,"::"+link_name) ) {
            return model_links[i];
        }
    }

    return gazebo::physics::LinkPtr();
}


gazebo::math::Vector3 Projection_point(gazebo::math::Vector3 Plane_point, gazebo::math::Vector3 Plane_norm, gazebo::math::Vector3 Original_point, gazebo::math::Vector3 Projection_dir)
{
    double S;
    S = ( Plane_norm.Dot(Plane_point - Original_point) ) / ( Plane_norm.Dot(Projection_dir) ) ;
    gazebo::math::Vector3 Point_proj;
    Point_proj = Original_point + S * Projection_dir;
    return Point_proj;
}

gazebo::math::Pose Get_handle_pose(gazebo::physics::ModelPtr object_model, gazebo::math::Pose hand_pose, std::string link_name, double width, double height, double length)
{
    const double inch2meter = 0.0254;

    const double box_tolerance_length = 0.0;    //unit: meter the same direction as mentioned above
    const double box_tolerance_width = 0.1;     //unit: meter the same direction as mentioned above
    const double box_tolerance_height = 0.1;    //unit: meter the same direction as mentioned above

    double box_length = length;           //unit: inch measured along the x axis of the box reference frame
    double box_width = width;             //unit: inch measured along the y axis of the box reference frame
    double box_height = height;           //unit: inch measured along the z axis of the box reference frame


    double object_length, object_width, object_height;
    object_length = box_length * inch2meter;
    object_width = box_width * inch2meter;
    object_height = box_height * inch2meter;

    physics::LinkPtr object_link = getLinkInModel(object_model,"link");
    math::Pose object_pose = object_link->GetWorldCoGPose();

    math::Vector3 object_x, object_y, object_z, pos_diff;
    object_x = object_pose.rot.GetXAxis();
    object_y = object_pose.rot.GetYAxis();
    object_z = object_pose.rot.GetZAxis();

    pos_diff = hand_pose.pos - object_pose.pos;

    double threshold_length, threshold_width, threshold_height;
    threshold_length = object_length/2 + box_tolerance_length;
    threshold_width = object_width/2 + box_tolerance_width;
    threshold_height = object_height/2 + box_tolerance_height;

    double projection_x = std::abs(pos_diff.Dot(object_x));
    double projection_y = std::abs(pos_diff.Dot(object_y));
    double projection_z = std::abs(pos_diff.Dot(object_z));

    double hand_identifier;
    if (link_name == "LWrMot3")
    {
        hand_identifier = 1.0;        //using left hand
    }
    else if (link_name == "RWrMot3")
    {
        hand_identifier = -1.0;       //using right hand
    }
    else
    {
        std::cout << "The link name typed is not the name of a hand!";
    }

    if (projection_x<threshold_length && projection_y<threshold_width && projection_z<threshold_height)
    {
        std::cout << "projection_length: "<< projection_x << "  threshold_length: " << threshold_length << std::endl;
        std::cout << "projection_width: "<< projection_y << "  threshold_width: " << threshold_width << std::endl;
        std::cout << "projection_height: "<< projection_z << "  threshold_height: " << threshold_height << std::endl;
        std::cout << "the hand is close enough to the object to be operated!" << std::endl;

        math::Vector3 hand_O, hand_X, hand_O_dir;
        math::Vector3 plane_O_Z, plane_O_Z_, plane_O_Y, plane_O_Y_;
        math::Vector3 plane_O_Z_norm, plane_O_Z__norm, plane_O_Y_norm, plane_O_Y__norm;
        math::Vector3 proj_O_Z, proj_O_Z_, proj_O_Y, proj_O_Y_;

        hand_O = hand_pose.pos;
        hand_X = hand_pose.pos + hand_pose.rot.GetXAxis();
        hand_O_dir = -hand_identifier * hand_pose.rot.GetYAxis();

        plane_O_Z = object_pose.pos + (object_height/2) * object_z;
        plane_O_Z_ = object_pose.pos + (object_height/2) * (-object_z);
        plane_O_Y = object_pose.pos + (object_width/2) * object_y;
        plane_O_Y_ = object_pose.pos + (object_width/2) * (-object_y);

        plane_O_Z_norm = object_z;
        plane_O_Z__norm = -object_z;
        plane_O_Y_norm = object_y;
        plane_O_Y__norm = -object_y;

        proj_O_Z = Projection_point(plane_O_Z, plane_O_Z_norm, hand_O, hand_O_dir);
        proj_O_Z_ = Projection_point(plane_O_Z_, plane_O_Z__norm, hand_O, hand_O_dir);
        proj_O_Y = Projection_point(plane_O_Y, plane_O_Y_norm, hand_O, hand_O_dir);
        proj_O_Y_ = Projection_point(plane_O_Y_, plane_O_Y__norm, hand_O, hand_O_dir);

        std::map<int, double> plane_situation;
        math::Vector3 temp1, temp2, temp3, temp4;
        math::Vector3 temp1_2, temp2_2, temp3_2, temp4_2;
        double S1, S2, S3, S4;

        temp1 = proj_O_Z - plane_O_Z;
        temp1_2 = proj_O_Z - hand_O;
        S1 = temp1_2.Dot(hand_O_dir);
        if ( ( std::abs( temp1.Dot(object_y) ) < (width/2) ) && ( std::abs( temp1.Dot(object_x) ) < (length/2) ) && ( S1 > 0 ) )
        {
            plane_situation[1] = S1;
        }

        temp2 = proj_O_Z_ - plane_O_Z_;
        temp2_2 = proj_O_Z_ - hand_O;
        S2 = temp2_2.Dot(hand_O_dir);
        if ( ( std::abs( temp2.Dot(object_y) ) < (width/2) ) && ( std::abs( temp2.Dot(object_x) ) < (length/2) ) && ( S2 > 0 ) )
        {
            plane_situation[2] = S2;
        }

        temp3 = proj_O_Y - plane_O_Y;
        temp3_2 = proj_O_Y - hand_O;
        S3 = temp3_2.Dot(hand_O_dir);
        if ( ( std::abs( temp3.Dot(object_z) ) < (height/2) ) && ( std::abs( temp3.Dot(object_x) ) < (length/2) ) && ( S3 > 0 ) )
        {
            plane_situation[3] = S3;
        }

        temp4 = proj_O_Y_ - plane_O_Y_;
        temp4_2 = proj_O_Y_ - hand_O;
        S4 = temp4_2.Dot(hand_O_dir);
        if ( ( std::abs( temp4.Dot(object_z) ) < (height/2) ) && ( std::abs( temp4.Dot(object_x) ) < (length/2) ) && ( S4 > 0 ) )
        {
            plane_situation[4] = S4;
        }

        int plane_index = 0;
        double S_final = 10000.0;

        if (plane_situation.empty())
        {
             std::cout << "The grasp pose is Not available! Please move the hand to make the projection point of the hand center on the surfaces of the wooden bar." << std::endl;
             gazebo::math::Pose handle_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
             return handle_pose;
        }
        else
        {
            std::map<int, double>::iterator  iter;
            for(iter = plane_situation.begin(); iter != plane_situation.end(); iter++)
            {
                  if (iter->second < S_final)
                  {
                      plane_index = iter->first;
                      S_final = iter->second;
                  }
            }

            double S_O;
            math::Vector3 Proj_O, Proj_X;
            math::Vector3 Proj_x_axis, Proj_y_axis, Proj_z_axis, Proj_x_axis_temp;

            switch (plane_index)
            {
            case 1:
                S_O = plane_situation[1];
                Proj_O = hand_O + S_O * hand_O_dir;
                Proj_X = Projection_point(plane_O_Z, plane_O_Z_norm, hand_X, hand_O_dir);
                Proj_x_axis_temp = Proj_X - Proj_O;
                Proj_x_axis = Proj_x_axis_temp.Normalize();
                Proj_y_axis = hand_identifier * plane_O_Z_norm;
                Proj_z_axis = Proj_x_axis.Cross(Proj_y_axis);
                break;

            case 2:
                S_O = plane_situation[2];
                Proj_O = hand_O + S_O * hand_O_dir;
                Proj_X = Projection_point(plane_O_Z_, plane_O_Z__norm, hand_X, hand_O_dir);
                Proj_x_axis_temp = Proj_X - Proj_O;
                Proj_x_axis = Proj_x_axis_temp.Normalize();
                Proj_y_axis = hand_identifier * plane_O_Z__norm;
                Proj_z_axis = Proj_x_axis.Cross(Proj_y_axis);
                break;

            case 3:
                S_O = plane_situation[3];
                Proj_O = hand_O + S_O * hand_O_dir;
                Proj_X = Projection_point(plane_O_Y, plane_O_Y_norm, hand_X, hand_O_dir);
                Proj_x_axis_temp = Proj_X - Proj_O;
                Proj_x_axis = Proj_x_axis_temp.Normalize();
                Proj_y_axis = hand_identifier * plane_O_Y_norm;
                Proj_z_axis = Proj_x_axis.Cross(Proj_y_axis);
                break;

            case 4:
                S_O = plane_situation[4];
                Proj_O = hand_O + S_O * hand_O_dir;
                Proj_X = Projection_point(plane_O_Y_, plane_O_Y__norm, hand_X, hand_O_dir);
                Proj_x_axis_temp = Proj_X - Proj_O;
                Proj_x_axis = Proj_x_axis_temp.Normalize();
                Proj_y_axis = hand_identifier * plane_O_Y__norm;
                Proj_z_axis = Proj_x_axis.Cross(Proj_y_axis);
                break;

            }

            math::Matrix4 handle_pose_temp;
            handle_pose_temp[0][0] = Proj_x_axis.x;
            handle_pose_temp[1][0] = Proj_x_axis.y;
            handle_pose_temp[2][0] = Proj_x_axis.z;
            handle_pose_temp[3][0] = 0.0;

            handle_pose_temp[0][1] = Proj_y_axis.x;
            handle_pose_temp[1][1] = Proj_y_axis.y;
            handle_pose_temp[2][1] = Proj_y_axis.z;
            handle_pose_temp[3][1] = 0.0;

            handle_pose_temp[0][2] = Proj_z_axis.x;
            handle_pose_temp[1][2] = Proj_z_axis.y;
            handle_pose_temp[2][2] = Proj_z_axis.z;
            handle_pose_temp[3][2] = 0.0;

            handle_pose_temp[0][3] = 0.0;
            handle_pose_temp[1][3] = 0.0;
            handle_pose_temp[2][3] = 0.0;
            handle_pose_temp[3][3] = 1.0;

            math::Quaternion handle_ori = handle_pose_temp.GetRotation();

            gazebo::math::Pose handle_pose(Proj_O, handle_ori);

            return handle_pose;


        }


    }
    else{
        std::cout << "projection_length: "<< projection_x << "  threshold_length: " << threshold_length << std::endl;
        std::cout << "projection_width: "<< projection_y << "  threshold_width: " << threshold_width << std::endl;
        std::cout << "projection_height: "<< projection_z << "  threshold_height: " << threshold_height << std::endl;
        std::cout << "the hand is NOT close enough to the object to be operated!" << std::endl;
        gazebo::math::Pose handle_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        return handle_pose;
    }

}


bool gazebo::GazeboYarpObjects::attach(const std::string& link_name, const std::string& object_name, const double width, const double height, const double length)
{
    if (joints_attached.count(object_name+"_attached_joint"))
    {
        std::cout<<"objects is already attached!!"<<std::endl;
        return false;
    }
    physics::ModelPtr object_model = m_world->GetModel(object_name);

    if( !object_model )
    {
        std::cout<<"could not get the model for "<<object_name<<"!!"<<std::endl;
        return false;
    }

    physics::LinkPtr parent_link = getLinkInModel(m_model,link_name);

    if( !parent_link ) {
        std::cout<<"could not get the links for "<<parent_link<<std::endl;
        return false;
    }


    math::Pose handle_pose = Get_handle_pose(object_model, parent_link->GetWorldCoGPose(), link_name, width, height, length);

    if( handle_pose.pos.x == 0.0 && handle_pose.pos.y == 0.0 && handle_pose.pos.z == 0.0 && handle_pose.rot.GetRoll() == 0.0 && handle_pose.rot.GetPitch() == 0.0 && handle_pose.rot.GetYaw() == 0.0 ) {
        std::cout<<"The hand posture is NOT suitable to grasp the specified object" <<std::endl;
        return false;
    }
    else
    {


        physics::ModelPtr object_handle_model = m_world->GetModel("object_handle");
        physics::LinkPtr object_handle_link = getLinkInModel(object_handle_model,"object_handle_link");

        object_handle_link->SetWorldPose(handle_pose);



        physics::JointPtr object_joint;
        object_joint = m_world->GetPhysicsEngine()->CreateJoint("revolute", object_model);
        if( !object_joint ) {
            std::cout<<"could not create joint for the object!!"<<std::endl;
            return false;
        }

        physics::LinkPtr object_link = getLinkInModel(object_model,"link");

        object_joint->SetName(object_name+"_attached_handle_joint");
        joints_attached[object_name+"_attached_handle_joint"]=object_joint;
        object_joint->Load(object_link, object_handle_link, math::Pose());
        object_joint->Attach(object_link, object_handle_link);
        object_joint->SetHighStop(0, 0);
        object_joint->SetLowStop(0, 0);


        //TODO add mutex
        physics::JointPtr joint;
        joint = m_world->GetPhysicsEngine()->CreateJoint("revolute", m_model);
        if( !joint ) {
            std::cout<<"could not create joint!!"<<std::endl;
            return false;
        }


        parent_link->SetCollideMode("none");
        attached_links[object_name]=parent_link;

        math::Pose hand_pose = parent_link->GetWorldCoGPose();
        object_handle_link->SetWorldPose(hand_pose);


        joint->SetName(object_name+"_attached_joint");
        joints_attached[object_name+"_attached_joint"]=joint;
        joint->Load(parent_link, object_handle_link, math::Pose());
        joint->Attach(parent_link, object_handle_link);
        joint->SetHighStop(0, 0);
        joint->SetLowStop(0, 0);
        //joint->SetParam("cfm", 0, 0);

        return true;
    }
}

bool gazebo::GazeboYarpObjects::detach(const std::string& object_name)
{
    physics::JointPtr joint, handle_joint;
    if (joints_attached.count(object_name+"_attached_joint"))
    {
        joint=joints_attached[object_name+"_attached_joint"];
    }
    else
    {
        return false;
    }

    if (joints_attached.count(object_name+"_attached_handle_joint"))
    {
        handle_joint=joints_attached[object_name+"_attached_handle_joint"];
    }
    else
    {
        return false;
    }


    //TODO add mutex
    joint->Detach();
    joints_attached.erase(object_name+"_attached_joint");

    handle_joint->Detach();
    joints_attached.erase(object_name+"_attached_handle_joint");

    if (attached_links.count(object_name))
        attached_links[object_name]->SetCollideMode("all");

    return true;
}


}
