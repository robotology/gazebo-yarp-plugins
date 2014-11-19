/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef GAZEBOYARP_TIMESYNCHRONIZERPLUGIN_HH
#define GAZEBOYARP_TIMESYNCHRONIZERPLUGIN_HH

#include <gazebo/common/Plugin.hh>

namespace gazebo
{
  class GazeboYarpTimeSynchronizer : public SystemPlugin
  {
  public:
        GazeboYarpTimeSynchronizer();
        virtual ~GazeboYarpTimeSynchronizer();
        
        virtual void Load(int _argc = 0, char **_argv = NULL);
  };
}


#endif