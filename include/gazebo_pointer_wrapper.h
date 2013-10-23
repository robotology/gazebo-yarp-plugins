#ifndef _GAZEBO_WORLD_H_
#define _GAZEBO_WORLD_H_

#include <gazebo/physics/physics.hh>
#include <yarp/sig/Vector.h>

class gazebo_pointer_wrapper
{
    static gazebo::physics::ModelPtr modelPtr;
    static yarp::sig::Vector ref_pos;

public:

    static gazebo::physics::ModelPtr getModel()
    {
        return modelPtr;
    }

    static void setModel(gazebo::physics::ModelPtr p)
    {
        modelPtr = p;
    }

};

#endif
