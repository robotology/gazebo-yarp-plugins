#include "GazeboYarpSkin.hh"

using namespace gazebo;
using namespace transport;

using yarp::os::Bottle;
using yarp::os::Network;
using yarp::os::Port;

GZ_REGISTER_MODEL_PLUGIN(GazeboYarpSkin)

namespace gazebo {

GazeboYarpSkin::GazeboYarpSkin() : ModelPlugin()
{  
}

GazeboYarpSkin::~GazeboYarpSkin()
{
    // Close YARP ports
    for (int i = 0; i < N_BODY_PARTS; i++) {
        this->outputPorts[i].close();
    }
}

void GazeboYarpSkin::updateLastSendTime()
{
    this->lastSendTime = this->model->GetWorld()->RealTime();
}

void GazeboYarpSkin::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
    // Initialize YARP network
    // yarp::os::Network::init();
    // if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout)) {
    //     yError() << "GazeboYarpSkin::Load error: yarp network does not seem to be available, is the yarpserver running?";
    //     return;
    // }

    // Store reference to the model
    this->model = _parent;

    // Set time of last sent skin data
    this->updateLastSendTime();

    // Setup YARP ports
    for (int i = 0; i < N_BODY_PARTS; i++) {
        this->outputPorts[i].open(this->portNames[i]);
        this->skin[i].resize(this->taxelCounts[i]);
        std::fill(this->skin[i].begin(), this->skin[i].end(), TAXEL_DEFAULT_VALUE);
    }

    // Listen to the update event
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboYarpSkin::OnUpdate, this)
    );

    // Subscribe to ~/physics/contacts Gazebo topic
    NodePtr node(new Node());
    node->Init();
    this->sub = node->Subscribe("~/physics/contacts", &GazeboYarpSkin::OnContact, this);
}

void GazeboYarpSkin::OnContact(ConstContactsPtr &_msg)
{
    if (_msg->contact_size() > 0) {
        int timeSec = _msg->time().sec();
        int timeNsec = _msg->time().nsec();

        for (unsigned int i = 0; i < _msg->contact_size(); ++i) {
            std::cmatch cm;
            std::string collisionName1 = _msg->contact(i).collision1();
            std::string collisionName2 = _msg->contact(i).collision2();
      
            for (int j = 0; j < N_BODY_PARTS; j++) {
                std::regex_match(collisionName1.c_str(), cm, std::regex(this->collisionPatterns[j]));
                if (!cm.size()) {
                    std::regex_match(collisionName2.c_str(), cm, std::regex(this->collisionPatterns[j]));
                }
                if (cm.size()) {
                    int taxelNumber = stoi(cm[1]);
                    msgs::Vector3d f = _msg->contact(i).wrench(0).body_1_wrench().force();
                    double force = sqrt(f.x() * f.x() + f.y() * f.y() + f.z() * f.z());
                    double taxelValue = std::min(
                        255.0,
                        ((force / MAX_TAXEL_FORCE) * 255.0)
                    );
                    this->skin[j][taxelNumber] = taxelValue;
                }
            }
        }
    }
}

void GazeboYarpSkin::OnUpdate()
{
    // Get time difference from last skin data
    common::Time diff = this->model->GetWorld()->RealTime() - this->lastSendTime;
  
    // Send skin data at 10 Hz
    // Note: for 50 Hz, use nsec value 20000000
    if (diff.sec > 0 || diff.nsec >= 100000000) {
        // Update last time skin update was sent
        this->updateLastSendTime();

        // Send YARP data
        for (int i = 0; i < N_BODY_PARTS; i++) {
            Bottle bot;
            for (int j = 0; j < this->taxelCounts[i]; j++) {
                bot.addFloat64(this->skin[i][j]);
            }
            this->outputPorts[i].write(bot);

            // Reset taxel data
            std::fill(this->skin[i].begin(), this->skin[i].end(), TAXEL_DEFAULT_VALUE);
        }
    }
}

}
