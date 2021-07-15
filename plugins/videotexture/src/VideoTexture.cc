/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia & iCub Facility
 * Authors: Marco Randazzo <marco.randazzo@iit.it>
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>

#include <gazebo/sensors/CameraSensor.hh>
#include <VideoTexture.hh>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/cv/Cv.h>

using namespace Ogre;
namespace gazebo 
{
  VideoTexture::VideoTexture()
  {
    m_network = 0;
  }

  VideoTexture::~VideoTexture()
  {
      m_connection.reset();
      if (m_network)
      {
        delete m_network;
        m_network = 0;
      }
      m_VideoPort.close();
      if(m_material.get())
      {
        m_material->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setBlank();
      }
      Ogre::TextureManager::getSingleton().remove(m_texture->getName());
  }

    void VideoTexture::Update()
    {
        ImageType* img = m_VideoPort.read(false);
        if(img)
        {
            typedef Ogre::HardwarePixelBufferSharedPtr pixelBuff;

            cv::Mat           image, converted_image;
            const cv::Mat*    image_ptr;
            pixelBuff         pixelBuffer;

            image       = yarp::cv::toCvMat(*img);
            image_ptr   = &image;
            pixelBuffer = this->m_texture->getBuffer();

            if (image_ptr->rows != m_height || image_ptr->cols != m_width)
            {
               cv::resize(*image_ptr, converted_image, cv::Size(m_width, m_height));
               image_ptr = &converted_image;
            }

            PixelBox pb = PixelBox(m_width, m_height, 1, FORMAT, (void*)image_ptr->data);
            m_texture->getBuffer()->blitFromMemory(pb);
        }
    }

  void VideoTexture::Load(rendering::VisualPtr parent, sdf::ElementPtr sdf) 
  {
    if (m_network != 0) return;

    m_network = new yarp::os::Network();
    m_model   = parent;

    if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout))
    {
       yError() << "VideoTexture::Load error: yarp network does not seem to be available, is the yarpserver running?";
       return;
    }

    //Getting .ini configuration file from sdf
    bool configuration_loaded = false;

    if(!sdf->HasElement("yarpConfigurationFile") && sdf->HasElement("sdf"))
    {
        sdf = sdf->GetElement("sdf");
    }

    if (sdf->HasElement("yarpConfigurationFile"))
    {
        std::string ini_file_name = sdf->Get<std::string>("yarpConfigurationFile");
        std::string ini_file_path = gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);

        if (ini_file_path != "" && m_parameters.fromConfigFile(ini_file_path.c_str()))
        {
            //yInfo() << "Found yarpConfigurationFile: loading from " << ini_file_path ;
            configuration_loaded = true;
        }
     }

    if (!configuration_loaded)
    {
        yError() << "VideoTexture : File .ini not found, load failed." ;
        return;
    }

    std::string sourcePortName;
    
#if GAZEBO_MAJOR_VERSION >= 8
    m_texName      = m_model->Name();
#else
    m_texName      = m_model->GetName();
#endif

    m_VideoPort.open("/"+m_texName);
    m_VideoPort.setReadOnly();

    m_connection   = event::Events::ConnectPreRender(std::bind(&VideoTexture::Update, this));
    m_width        = 480;
    m_height       = 640;
    m_scale        = 1;
    m_width        = m_parameters.find("widthRes").asInt32();
    m_height       = m_parameters.find("heightRes").asInt32();
    m_scale        = m_parameters.find("heightLen").asFloat64();
    m_material     = Ogre::MaterialManager::getSingleton().getByName(m_model->GetMaterialName());
    m_texture      = Ogre::TextureManager::getSingleton().getByName(m_texName);

    if(m_texture.isNull())
    {
        m_texture = Ogre::TextureManager::getSingleton().createManual
                  (
                      m_texName,
                      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                      Ogre::TEX_TYPE_2D,
                      m_width, m_height,
                      0,
                      FORMAT,
                      Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE
                  );
    }

    if(m_parameters.check("defaultSourcePortName"))
    {
        sourcePortName = m_parameters.find("defaultSourcePortName").asString();
        if(!sourcePortName.empty() && yarp::os::NetworkBase::exists(sourcePortName))
        {
            yarp::os::NetworkBase::connect(sourcePortName, "/"+m_texName);
            Update();
        }
    }
    else
    {
        unsigned char* data = new unsigned char[m_width * m_height * 3];
        for(size_t i = 0; i < m_width * m_height * 3; i++)
        {
            data[i] = std::rand() % 255;
        }
        m_texture->getBuffer()->blitFromMemory(PixelBox(m_width, m_height, 1, FORMAT, (void*)data));
    }


    if(m_material.get())
    {
        m_material.get()->getTechnique(0)->getPass(0)->removeAllTextureUnitStates();
        m_material.get()->getTechnique(0)->getPass(0)->createTextureUnitState(m_texture->getName());
        m_material.get()->setReceiveShadows(false);
    }    

    //double wScale = m_scale*(float(m_width)/float(m_height));
    //m_model->GetSceneNode()->scale(wScale, m_scale, m_scale);
    //m_model->GetSceneNode()->translate(0, m_scale/2, 0);

  }

  GZ_REGISTER_VISUAL_PLUGIN(VideoTexture);
}
