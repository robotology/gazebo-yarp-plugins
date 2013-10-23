#ifndef _GAZEBO_WORLD_H_
#define _GAZEBO_WORLD_H_

#include <gazebo/physics/physics.hh>
#include <yarp/sig/Vector.h>

class gazebo_world
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

    static void set_ref_pos(yarp::sig::Vector p)
    {
        ref_pos = p;
    }

    static yarp::sig::Vector get_ref_pos()
    {
        return ref_pos;
    }
};

#endif
