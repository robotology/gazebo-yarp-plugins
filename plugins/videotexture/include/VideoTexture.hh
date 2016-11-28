/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: Marco Randazzo <marco.randazzo@iit.it>
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBO_VIDEO_VISUAL_H
#define GAZEBO_VIDEO_VISUAL_H

#include <boost/thread/mutex.hpp>
#include <opencv2/opencv.hpp>

#include <yarp/sig/Image.h>
#include <yarp/os/Port.h>
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/transport/TransportTypes.hh>

typedef yarp::sig::ImageOf<yarp::sig::PixelRgb> ImageType;
#define FORMAT Ogre::PF_B8G8R8

namespace gazebo 
{

  class VideoTexture : public VisualPlugin 
  {
      void Update();
    public: 
    
      VideoTexture();
      virtual ~VideoTexture();
      void Load(rendering::VisualPtr parent, sdf::ElementPtr sdf);

    protected:
      yarp::os::BufferedPort<ImageType> m_VideoPort;

      std::string            m_texName;
      Ogre::MaterialPtr      m_material;
      Ogre::TexturePtr       m_texture;
      event::ConnectionPtr   m_connection;
      rendering::VisualPtr   m_model;
      yarp::os::Property     m_parameters;
      boost::mutex           m_image;
      int                    m_height;
      int                    m_width;
      double                 m_scale;
      yarp::os::Network*     m_network;
      yarp::os::Port         m_input_port;
      std::string            m_port_name;
  };

}

#endif
