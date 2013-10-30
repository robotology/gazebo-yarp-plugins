/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
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

  if (_sdf->GetElement("boardId")) {
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
    if(this->yarpFTsensor!=NULL) {
        yarp::sig::Vector wrench; wrench.size(6);
        wrench[0] = _msg.wrench().force().x();
        wrench[1] = _msg.wrench().force().y();
        wrench[2] = _msg.wrench().force().z();
        wrench[3] = _msg.wrench().torque().x();
        wrench[4] = _msg.wrench().torque().y();
        wrench[5] = _msg.wrench().torque().z();
        this->yarpFTsensor->data = wrench;
    }
}
}
