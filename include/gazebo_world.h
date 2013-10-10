#ifndef _GAZEBO_WORLD_H_
#define _GAZEBO_WORLD_H_

#include <gazebo/physics/physics.hh>

class gazebo_world
{
    static gazebo::physics::WorldPtr worldPtr;
    static gazebo::physics::ModelPtr modelPtr;

public:
    static gazebo::physics::WorldPtr getWorld()
    {
        return worldPtr;
    }

    static gazebo::physics::ModelPtr getModel()
    {
        return modelPtr;
    }

    static void setWorld(gazebo::physics::WorldPtr p)
    {
        worldPtr = p;
    }

    static void setModel(gazebo::physics::ModelPtr p)
    {
        modelPtr = p;
    }
};

#endif
