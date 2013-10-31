#include "fakebotIMUplugin.hh"
#include <yarp/dev/ServerInertial.h>
#include <yarp/dev/PolyDriver.h>

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(fakebotIMUplugin)

#define toDeg(X) (X*180.0/M_PI)

fakebotIMUplugin::fakebotIMUplugin() : SensorPlugin(),
    _yarp(),
    _p(),
    _bot()
{
    _p.open("/coman/inertial:o");
}

void fakebotIMUplugin::Init()
{
    std::cout<<"*** fakebotIMUplugin started ***"<<std::endl;
    if (!_yarp.checkNetwork())
        std::cout<<"Sorry YARP network does not seem to be available, is the yarp server available?"<<std::endl;
    else
        std::cout<<"YARP Server found!"<<std::endl;
}

fakebotIMUplugin::~fakebotIMUplugin()
{
    std::cout<<"IMU says: Goodbye!"<<std::endl;
}

void fakebotIMUplugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    std::cout<<"Sensor Name:"<<_sensor->GetName()<<std::endl;
    this->parentSensor =
        boost::shared_dynamic_cast<sensors::ImuSensor>(_sensor);

    if (!this->parentSensor)
    {
        gzerr << "fakebotPlugin requires a IMUSensor.\n";
        return;
    }

    this->updateConnection = this->parentSensor->ConnectUpdated(
          boost::bind(&fakebotIMUplugin::OnUpdate, this));

    this->parentSensor->SetActive(true);
}

/**
 *
 * Export an inertial sensor.
 * The network interface is a single Port.
 * We will stream bottles with 12 floats:
 * 0  1   2  = Euler orientation data (Kalman filter processed)
 * 3  4   5  = Calibrated 3-axis (X, Y, Z) acceleration data
 * 6  7   8  = Calibrated 3-axis (X, Y, Z) gyroscope data
 * 9 10 11   = Calibrated 3-axis (X, Y, Z) magnetometer data
 *
 */
void fakebotIMUplugin::OnUpdate()
{
    msgs::IMU imu_data;
    imu_data = this->parentSensor->GetImuMessage();

    gazebo::msgs::Quaternion quaternion_msg = *(imu_data.mutable_orientation());
    gazebo::math::Quaternion orient;
    orient.Set(quaternion_msg.w(), quaternion_msg.x(), quaternion_msg.y(), quaternion_msg.z());

    _bot.addDouble( toDeg(orient.GetRoll()));
    _bot.addDouble( toDeg(orient.GetPitch()));
    _bot.addDouble( toDeg(orient.GetYaw()));
//    std::cout<<"Orientation: [ "<<orient.GetRoll()<<" "<<
//               rient.GetPitch()<<" "<<toDeg(orient.GetYaw()<<std::endl;

    _bot.addDouble(imu_data.mutable_linear_acceleration()->x());
    _bot.addDouble(imu_data.mutable_linear_acceleration()->y());
    _bot.addDouble(imu_data.mutable_linear_acceleration()->z());
//    std::cout<<"Linear Acceleration: [ "<<imu_data.mutable_linear_acceleration()->x()<<" "<<
//               imu_data.mutable_linear_acceleration()->y()<<" "<<
//               imu_data.mutable_linear_acceleration()->z()<<std::endl;

    _bot.addDouble( toDeg(imu_data.mutable_angular_velocity()->x()) );
    _bot.addDouble( toDeg(imu_data.mutable_angular_velocity()->y()) );
    _bot.addDouble( toDeg(imu_data.mutable_angular_velocity()->z()) );
//    std::cout<<"Angular Velocity: [ "<<imu_data.mutable_linear_acceleration()->x()<<" "<<
//               imu_data.mutable_linear_acceleration()->y()<<" "<<
//               imu_data.mutable_linear_acceleration()->z()<<std::endl;

    _bot.addDouble( toDeg(orient.GetRoll()));
    _bot.addDouble( toDeg(orient.GetPitch()));
    _bot.addDouble( toDeg(orient.GetYaw()));

    yarp::os::Stamp ts;
    ts = yarp::os::Stamp(0, imu_data.mutable_stamp()->sec());
    _p.setEnvelope(ts);

    _p.write(_bot);
    _bot.clear();
}
