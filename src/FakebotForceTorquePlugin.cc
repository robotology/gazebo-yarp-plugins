/*
* Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
* Authors: Mingo Enrico, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/


#include <string>
#include <algorithm>
#include <stdlib.h>

#include <yarp/os/Network.h>
#include <gazebo/transport/Node.hh>
#include <gazebo/common/Assert.hh>

#include <FakebotForceTorquePlugin.hh>
#include <fakebotFTsensor.h>
#include <analogServer.h>

using std::string;

namespace gazebo
{
GZ_REGISTER_SENSOR_PLUGIN(FakebotForceTorquePlugin)

/*
 * We have one yarpFTsensor (yarp::dev::IAnalogSensor) and one factory for all FT sensors;
 * still we have one AnalogServer for each FT sensor that reads just one portion of
 * the unique yarpFTsensor AnalogSensor, and publishes it.
 */
unsigned int FakebotForceTorquePlugin::iBoards = 0;

/////////////////////////////////////////////////
FakebotForceTorquePlugin::FakebotForceTorquePlugin() : _server(NULL), yarpFTsensor(NULL), _yarp()
{
    this->iBoards++;

    if(this->yarpFTsensor == NULL) {
        yarp::dev::DriverCreator *fakebotFtsensor_factory =
                new yarp::dev::DriverCreatorOf<yarp::dev::fakebotFTsensor>("fakebotFTsensor","FTsensor","fakebotFTsensor");
        yarp::dev::Drivers::factory().add(fakebotFtsensor_factory); // hand factory over to YARP
        yarp::os::Property _parameters;
        _parameters.put("device", "fakebotFTsensor");
        _parameters.put("subdevice", "fakebotFTsensor");
        _parameters.put("name", "/coman/FTsensor");
        this->_driver.open(_parameters);

        if (!this->_driver.isValid())
           fprintf(stderr, "Device did not open\n");
        else {
            this->_driver.view(this->yarpFTsensor);
            printf("Device fakebotFTsensor initialized correctly\n");
        }
    }
}

/////////////////////////////////////////////////
FakebotForceTorquePlugin::~FakebotForceTorquePlugin()
{
    if(this->_server!= NULL && this->_server->isRunning()) {
        this->_server->stop();
    }

    if(this->parentSensor != NULL) {
        printf("FakebotForceTorquePlugin: Disconnecting from sensor %s\n",
               this->parentSensor->GetName().c_str());
        this->parentSensor->DisconnectUpdate(this->connection);
        this->parentSensor.reset();
    }

    if(this->yarpFTsensor != NULL && this->_driver.isValid()) {
        this->yarpFTsensor = NULL;
        this->_driver.close();
    }
}

/////////////////////////////////////////////////
void FakebotForceTorquePlugin::Load(sensors::SensorPtr _parent,
    sdf::ElementPtr _sdf)
{
  this->parentSensor =
    boost::dynamic_pointer_cast<sensors::ForceTorqueSensor>(_parent);

  if (!this->parentSensor)
    gzthrow("ForceTorquePlugin requires a force_torque sensor as its parent.");

  this->connection = this->parentSensor->ConnectUpdate(
        boost::bind(&FakebotForceTorquePlugin::OnUpdate, this, _1));

  this->world = gazebo::physics::get_world(this->parentSensor->GetWorldName());

  std::string jointName = this->parentSensor->GetParentName();
  physics::BasePtr parentEntity = world->GetByName(jointName);
  if(!parentEntity)
      gzthrow("Could not get parent joint.");

  physics::JointPtr parentJoint;
  parentJoint = boost::dynamic_pointer_cast<physics::Joint>(parentEntity);
  // link where the ft sensor is physically located
  this->ftLink = parentJoint->GetChild();
  if(!this->ftLink)
      gzthrow("Could not get child link of parent joint.");

  if (_sdf->HasElement("boardId")) {
    this->boardId = _sdf->GetElement("boardId")->GetValueUInt();
      if(this->boardId > this->iBoards)
        this->iBoards = this->boardId;
  } else
    this->boardId = this->iBoards;

  if(this->yarpFTsensor!=NULL) {
    if (!_yarp.checkNetwork())
      std::cout<<"Sorry YARP network does not seem to be available, is the yarp server available?"<<std::endl;
    else {
        yarp::os::Property prop;
        prop.put("device", "analogServer");
        prop.put("robotName", "coman");
        prop.put("deviceId", this->parentSensor->GetName().c_str());
        prop.put("rate", 1);

        /* TODO: when analogServer will be included in YARP, this code needs to be used
        yarp::dev::PolyDriver poly;


        poly.open(prop);
        if(!poly.isValid())
          printf("error opening analogServer");

        poly.view(this->_server);
        */
        this->_server = new yarp::dev::AnalogServer();
        if(this->_server != NULL) {
            if(!this->_server->open(prop))
                printf("error opening analogServer");
            else {
                this->_server->attach(this->yarpFTsensor);
                this->_server->start();
            }
        } else
            printf("error creating analogServer");
    }

    printf("----- FakebotForceTorquePlugin:");
    printf("Instantiated FT plugin with boardId %d ",this->boardId);
    printf("for sensor %s\n",this->parentSensor->GetName().c_str());
  }
}

/////////////////////////////////////////////////
void FakebotForceTorquePlugin::OnUpdate(msgs::WrenchStamped _msg)
{

    // ft force and torque measurements in joint (child link) frame
    math::Vector3 l_ftForce( _msg.wrench().force().x(),
                             _msg.wrench().force().y(),
                             _msg.wrench().force().z());
    math::Vector3 l_ftTorque( _msg.wrench().torque().x(),
                              _msg.wrench().torque().y(),
                              _msg.wrench().torque().z());

    // ft force and torque measurements in world frame
    math::Vector3 w_ftForce;
    math::Vector3 w_ftTorque;

    // ft force and torque measurements in ft frame
    math::Vector3 ftForce;
    math::Vector3 ftTorque;

    // frame of reference for the ft readings, expressed in world coords
    math::Pose w_LinkPose = this->ftLink->GetWorldPose();

    // ft pose, expressed in link coords
    math::Pose l_ftPose = this->parentSensor->GetPose();

    // ft pose, expressed in world coords
    math::Pose w_ftPose = l_ftPose+w_LinkPose;

    // from link reference frame to ft reference frame, expressed in world coords
    math::Vector3 w_lft = w_LinkPose.rot.RotateVector(l_ftPose.pos);

    // rotate ft force from link to world coords
    w_ftForce = w_LinkPose.rot.RotateVector(l_ftForce);
    // translate ft torque pivot to ft frame, express it in world frame coords
    w_ftTorque = w_LinkPose.rot.RotateVector(l_ftTorque);
    w_ftTorque += w_ftForce.Cross(w_lft);

    // rotate ft force from world to ft coords
    ftForce = w_ftPose.rot.RotateVectorReverse(w_ftForce);

    // rotate ft torque from world to ft coords
    ftTorque = w_ftPose.rot.RotateVectorReverse(w_ftTorque);

//    static unsigned long int _clock = 0;
//    if(_clock%10000 == 0) {
//        std::cout << "----------------------" << std::endl;
//        std::cout << "l2ft " << w_lft[0] << " " << w_lft[1] << " " << w_lft[2] << std::endl;
//        std::cout << "l  f " << l_ftForce[0] << " " << l_ftForce[1] << " " << l_ftForce[2] << std::endl;
//        std::cout << "l  t " << l_ftTorque[0] << " " << l_ftTorque[1] << " " << l_ftTorque[2] << std::endl;

//        std::cout << "ft f " << ftForce[0] << " " << ftForce[1] << " " << ftForce[2] << std::endl;
//        std::cout << "ft t " << ftTorque[0] << " " << ftTorque[1] << " " << ftTorque[2] << std::endl;
//    }
//    _clock++;

    if(this->yarpFTsensor!=NULL) {
        this->yarpFTsensor->mutex.wait();
        yarp::sig::Vector wrench; wrench.size(6);
        wrench[0] = ftForce[0];
        wrench[1] = ftForce[1];
        wrench[2] = ftForce[2];
        wrench[3] = ftTorque[0];
        wrench[4] = ftTorque[1];
        wrench[5] = ftTorque[2];
        this->yarpFTsensor->data = wrench;
        this->yarpFTsensor->mutex.post();
    }
}
}
