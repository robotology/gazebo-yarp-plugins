#ifndef GAZEBOYARP_SKIN_HH
#define GAZEBOYARP_SKIN_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Subscriber.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <yarp/os/Bottle.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Time.h>

#include <string>
#include <iostream>
#include <regex>
#include <math.h>
#include <cstring>

#define N_BODY_PARTS 7
#define N_TAXELS_TORSO 768
#define N_TAXELS_ARM 768
#define N_TAXELS_FOREARM 384
#define N_TAXELS_HAND 192
#define MAX_TAXEL_FORCE 0.1
#define TAXEL_DEFAULT_VALUE 0.0

using yarp::os::Bottle;
using yarp::os::Network;
using yarp::os::Port;

namespace gazebo
{
    /// \class GazeboYarpSkin
    /// Gazebo Plugin emulating the iCub skin in Gazebo.
    /// 
    /// This model plugin subscribes to ~/physics/contacts Gazebo topic,
    /// filters out skin contacts using regular expression on contact names,
    /// and forwards all detected skin contacts to corresponding YARP ports.

    class GazeboYarpSkin : public ModelPlugin
    {
    public:
        GazeboYarpSkin();
        virtual ~GazeboYarpSkin();
        virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        void OnContact(ConstContactsPtr &_msg);

    private:
        physics::ModelPtr model;
        event::ConnectionPtr updateConnection;
        transport::SubscriberPtr sub;
        common::Time lastSendTime;
        std::vector<double> skin[N_BODY_PARTS];
        int taxelCounts[N_BODY_PARTS] = {
            N_TAXELS_ARM,
            N_TAXELS_FOREARM,
            N_TAXELS_HAND,
            N_TAXELS_ARM,
            N_TAXELS_FOREARM,
            N_TAXELS_HAND,
            N_TAXELS_TORSO
        };
        std::string portNames[N_BODY_PARTS] = {
            "/icubSim/skin/left_arm_comp",
            "/icubSim/skin/left_forearm_comp",
            "/icubSim/skin/left_hand_comp",
            "/icubSim/skin/right_arm_comp",
            "/icubSim/skin/right_forearm_comp",
            "/icubSim/skin/right_hand_comp",
            "/icubSim/skin/torso_comp"
        };
        std::string collisionPatterns[N_BODY_PARTS] = {
            "iCub::left_upper_arm_skin::left_upper_arm_skin_collision_([0-9]+).*",
            "iCub::left_forearm_skin::left_forearm_skin_collision_([0-9]+).*",
            "iCub::left_hand_skin::left_hand_skin_collision_([0-9]+).*",
            "iCub::right_upper_arm_skin::right_upper_arm_skin_collision_([0-9]+).*",
            "iCub::right_forearm_skin::right_forearm_skin_collision_([0-9]+).*",
            "iCub::right_hand_skin::right_hand_skin_collision_([0-9]+).*",
            "iCub::chest_skin::chest_skin_collision_([0-9]+).*"
        };
        Port outputPorts[N_BODY_PARTS];
        virtual void OnUpdate();
        void updateLastSendTime();
  };
}
#endif

