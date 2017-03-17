#ifndef YARPGAZEBO_EXTERNALWRENCHSERVER_H
#define YARPGAZEBO_EXTERNALWRENCHSERVER_H

#include <externalwrench.h>

namespace gazebo
{
    class ExternalWrenchServer: public yarp::os::Thread
    {   
    private:

    
    public:
        virtual bool        threadInit();
        virtual void        run();
        virtual void        threadRelease();
    };
}

#endif
