#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <gazebo/gazebo_client.hh> //only if version is > 6
#include <iostream>

int main(int argc, char** argv) {
  gazebo::client::setup(argc, argv);

  //Create node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  std::string topicName = "/gazebo/"+ std::string(argv[1]) + "/add_goal";
  gazebo::transport::PublisherPtr pub =
    node->Advertise<gazebo::msgs::Vector3d>(topicName);

  std::cout << "Waiting for connection to " << topicName << "...\n";
  pub->WaitForConnection();
  std::cout << "Connection made!\n";

  gazebo::msgs::Vector3d msg;

  gazebo::msgs::Set(&msg, ignition::math::Vector3d(std::atof(argv[2]), std::atof(argv[3]), 1.25));
  std::cout << "Attempting to publish...\n";
  pub->Publish(msg);
  std::cout << "Published!\n";
  gazebo::client::shutdown();
}