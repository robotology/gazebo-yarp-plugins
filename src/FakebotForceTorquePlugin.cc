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

#include <gazebo/transport/Node.hh>
#include <gazebo/common/Assert.hh>

#include <FakebotForceTorquePlugin.hh>
#include <fakebotFTsensor.h>

using std::string;

namespace gazebo
{
GZ_REGISTER_SENSOR_PLUGIN(FakebotForceTorquePlugin)

yarp::dev::fakebotFTsensor* FakebotForceTorquePlugin::yarpFTsensor = NULL;
unsigned int FakebotForceTorquePlugin::iBoards = 0;

/////////////////////////////////////////////////
FakebotForceTorquePlugin::FakebotForceTorquePlugin()
{
    this->iBoards++;
    if(this->yarpFTsensor == NULL) {
        yarp::dev::DriverCreator *fakebotFtsensor_factory =
                new yarp::dev::DriverCreatorOf<yarp::dev::fakebotFTsensor>("fakebotFTsensor","FTsensor","fakebotFTsensor");
        yarp::dev::Drivers::factory().add(fakebotFtsensor_factory); // hand factory over to YARP
        _parameters.put("device", "FTsensor");
        _parameters.put("subdevice", "fakebotFTsensor");
        _parameters.put("name", "/fakebotFTsensor");
        this->_driver.open(_parameters);
        this->_driver.view(this->yarpFTsensor);
        if (!this->_driver.isValid())
           fprintf(stderr, "Device did not open\n");

        printf("Device initialized correctly, now sitting and waiting\n");
    }
}

/////////////////////////////////////////////////
FakebotForceTorquePlugin::~FakebotForceTorquePlugin()
{
  this->parentSensor->DisconnectUpdate(this->connection);
  this->parentSensor.reset();
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

  this->yarpFTsensor->bMap[this->boardId] = this->parentSensor->GetName();
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
        this->yarpFTsensor->data[this->parentSensor->GetName()] = wrench;
    }
  // TODO: do we need to write inside the fakebotFTsensor now?

}
}
