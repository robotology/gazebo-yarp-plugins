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

namespace gazebo 
{

  VideoVisual::VideoVisual( const std::string &name, rendering::VisualPtr parent, int height, int width) : 
      rendering::Visual(name, parent), m_height(height), m_width(width) 
  {

    m_texture = Ogre::TextureManager::getSingleton().createManual(
        name + "__VideoTexture__",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D,
        m_width, m_height,
        0,
        Ogre::PF_BYTE_BGRA,
        Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

    Ogre::MaterialPtr material =  Ogre::MaterialManager::getSingleton().create(name + "__VideoMaterial__", "General");
    material->getTechnique(0)->getPass(0)->createTextureUnitState(name + "__VideoTexture__");
    material->setReceiveShadows(false);

    double factor = 1.0;

    Ogre::ManualObject mo(name + "__VideoObject__");
    mo.begin(name + "__VideoMaterial__", Ogre::RenderOperation::OT_TRIANGLE_LIST);

    mo.position(-factor / 2, factor / 2, 0.51);
    mo.textureCoord(0, 0);

    mo.position(factor / 2, factor / 2, 0.51);
    mo.textureCoord(1, 0);

    mo.position(factor / 2, -factor / 2, 0.51);
    mo.textureCoord(1, 1);

    mo.position(-factor / 2, -factor / 2, 0.51);
    mo.textureCoord(0, 1);

    mo.triangle(0, 3, 2);
    mo.triangle(2, 1, 0);
    mo.end();

    mo.convertToMesh(name + "__VideoMesh__");

    Ogre::MovableObject *obj = (Ogre::MovableObject*)
    this->GetSceneNode()->getCreator()->createEntity( name + "__VideoEntity__", name + "__VideoMesh__");
    obj->setCastShadows(false);
    this->AttachObject(obj);
  }

  VideoVisual::~VideoVisual() {}

  void VideoVisual::onRead(ImageType &img)
  {
    cv::Mat matimage = cv::cvarrToMat( static_cast<IplImage*>(img.getIplImage()) ); 
    this->render(matimage);
    matimage.release();
  }
  
  void VideoVisual::render(const cv::Mat& image) 
  {
    const cv::Mat* image_ptr = &image;
    cv::Mat converted_image;
    if (image_ptr->rows != m_height || image_ptr->cols != m_width) 
    {
      cv::resize(*image_ptr, converted_image, cv::Size(m_width, m_height));
      image_ptr = &converted_image;
    }

    Ogre::HardwarePixelBufferSharedPtr pixelBuffer = this->m_texture->getBuffer();

    pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
    const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();
    uint8_t* pDest = static_cast<uint8_t*>(pixelBox.data);
    memcpy(pDest, image_ptr->data, m_height * m_width * 4);
    pixelBuffer->unlock();
  }

  VideoTexture::VideoTexture()
  {
    m_network =0;
  }

  VideoTexture::~VideoTexture()
  {
    if (m_network)
    {
      delete m_network;
      m_network=0;
    }
    if (m_video_visual)
    {
        delete m_video_visual;
        m_video_visual=0;
    }
  }

  void VideoTexture::Load(rendering::VisualPtr parent, sdf::ElementPtr sdf) 
  {
    yInfo()<<"VideoTexture plugin started";
    if (m_network!=0) return;

    m_network=new yarp::os::Network(); 

    if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout))
    {
       yError() << "VideoTexture::Load error: yarp network does not seem to be available, is the yarpserver running?";
       return;
    }

    m_model = parent;

    //Getting .ini configuration file from sdf
    bool configuration_loaded = false;

    if (sdf->HasElement("yarpConfigurationFile"))
    {
        std::string ini_file_name = sdf->Get<std::string>("yarpConfigurationFile");
        std::string ini_file_path = gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);

        if (ini_file_path != "" && m_parameters.fromConfigFile(ini_file_path.c_str()))
	{
            yInfo() << "Found yarpConfigurationFile: loading from " << ini_file_path ;
            configuration_loaded = true;
        }
     }

    if (!configuration_loaded)
    {
        yError() << "VideoTexture::Load error could not load configuration file";
        return;
    }

    std::string portname=m_parameters.find("name").asString();

    m_width=320;
    m_height=640;
    m_video_visual = new VideoVisual("visual_add_number_here", parent, m_height, m_width);
    m_video_visual->open("/videovisual");
    m_video_visual->setReadOnly();    
  }

  GZ_REGISTER_VISUAL_PLUGIN(VideoTexture);
}
