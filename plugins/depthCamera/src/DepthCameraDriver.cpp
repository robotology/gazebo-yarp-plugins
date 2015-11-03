/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alessandro Settimi and Marco Randazzo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "DepthCameraDriver.h"

#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>

#include <gazebo/math/Vector3.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/DepthCameraSensor.hh>

using namespace std;
using namespace yarp::dev;

const std::string YarpScopedName = "sensorScopedName";
double GazeboYarpDepthCameraDriver::start_time =0;

void GazeboYarpDepthCameraDriver::print (unsigned char* pixbuf, int pixbuf_w, int pixbuf_h, int x, int y, char* s, int size)
{
   int pixelsize =5;
   for (int i=0;i<size;i++)   
   {
      char* num_p=0;
      switch(s[i])
      {
	case '0' : num_p=num[0].data; break;
	case '1' : num_p=num[1].data; break;
	case '2' : num_p=num[2].data; break;
	case '3' : num_p=num[3].data; break;
	case '4' : num_p=num[4].data; break;
	case '5' : num_p=num[5].data; break;
	case '6' : num_p=num[6].data; break;
	case '7' : num_p=num[7].data; break;
	case '8' : num_p=num[8].data; break;
	case '9' : num_p=num[9].data; break;
	case ' ' : num_p=num[10].data; break;
        case '.' : num_p=num[11].data; break;
      }

      for (int yi=0;yi<5;yi++)   
      for (int xi=0;xi<3;xi++)
      {
         int ii=yi*3+xi;
         if (num_p[ii]=='*')
         {
            for (int r=yi*pixelsize; r<yi*pixelsize+pixelsize; r++)
	    {   
                int off = i*(pixelsize+20);
		for (int c=xi*pixelsize+off;  c<xi*pixelsize+pixelsize+off; c++)
		{
                    unsigned char *pixel = pixbuf + c*3 + r*(pixbuf_w*3);
	   	    pixel[0] = 0;
		    pixel[1] = 0;
		    pixel[2] = 255;
		}
            }
         }
      }
   }
}

GazeboYarpDepthCameraDriver::GazeboYarpDepthCameraDriver()
{
    m_vertical_flip     = false;
    m_horizontal_flip   = false;
    m_display_time_box  = false;
    m_display_timestamp = false;
    start_time = yarp::os::Time::now();
    counter=0;
    sprintf(num[0].data, "**** ** ** ****");
    sprintf(num[1].data, " *  *  *  *  * ");
    sprintf(num[2].data, "***  *****  ***");
    sprintf(num[3].data, "***  ****  ****");
    sprintf(num[4].data, "* ** ****  *  *");
    sprintf(num[5].data, "****  ***  ****");
    sprintf(num[6].data, "****  **** ****");
    sprintf(num[7].data, "***  *  *  *  *");
    sprintf(num[8].data, "**** ***** ****");
    sprintf(num[9].data, "**** ****  ****");
    sprintf(num[10].data,"               ");
    sprintf(num[11].data,"          ** **");
}


GazeboYarpDepthCameraDriver::~GazeboYarpDepthCameraDriver()
{
}

//DEVICE DRIVER
bool GazeboYarpDepthCameraDriver::open(yarp::os::Searchable& config)
{
    //Get gazebo pointers
    std::string sensorScopedName(config.find(YarpScopedName.c_str()).asString().c_str());

    std::cout << "GazeboYarpDepthCameraDriver::open() " << sensorScopedName << std::endl;

    m_imageCameraSensorPtr = (gazebo::sensors::CameraSensor*)GazeboYarpPlugins::Handler::getHandler()->getSensor(sensorScopedName);
    if (!m_imageCameraSensorPtr) {
        std::cout << "GazeboYarpDepthCameraDriver Error: camera sensor was not found" << std::endl;
        return false;
    }
    m_imageCameraPtr = m_imageCameraSensorPtr->GetCamera();


    m_depthCameraSensorPtr = (gazebo::sensors::DepthCameraSensor*)GazeboYarpPlugins::Handler::getHandler()->getSensor(sensorScopedName);
    if (!m_depthCameraSensorPtr) {
        std::cout << "GazeboYarpDepthCameraDriver Error: camera sensor was not found" << std::endl;
        return false;
    }
    m_depthCameraPtr = this->m_depthCameraSensorPtr->GetDepthCamera();


    if (config.check("vertical_flip")) m_vertical_flip =true;
    if (config.check("horizonal_flip")) m_horizontal_flip =true;
    if (config.check("display_timestamp")) m_display_timestamp =true;
    if (config.check("display_time_box")) m_display_time_box =true;

    m_width  = m_imageCameraPtr->GetImageWidth();
    m_height = m_imageCameraPtr->GetImageHeight();
    m_imageFrame_BufferSize = 3*m_width*m_height;
    m_depthFrame_BufferSize = m_width*m_height;
    m_RGBPointCloud_BufferSize = 0;

    m_dataMutex.wait();
    m_imageFrame_Buffer = new unsigned char[m_imageFrame_BufferSize];
    memset(m_imageFrame_Buffer, 0x00, m_imageFrame_BufferSize);

    m_depthFrame_Buffer = new unsigned char[m_depthFrame_BufferSize];
    memset(m_depthFrame_Buffer, 0x00, m_depthFrame_BufferSize);

    m_RGBPointCloud_Buffer = new unsigned char[m_RGBPointCloud_BufferSize];
    memset(m_RGBPointCloud_Buffer, 0x00, m_RGBPointCloud_BufferSize);

    m_lastTimestamp.update();
    m_dataMutex.post();

    //Connect the driver to the gazebo simulation
    this->m_updateImageFrame_Connection    = m_imageCameraPtr->ConnectNewImageFrame(boost::bind(
                                                                &GazeboYarpDepthCameraDriver::OnNewImageFrame,
                                                                this, _1, _2, _3, _4, _5));

    this->m_updateRGBPointCloud_Connection = m_depthCameraPtr->ConnectNewRGBPointCloud(boost::bind(
                                                                &GazeboYarpDepthCameraDriver::OnNewRGBPointCloud,
                                                                this, _1, _2, _3, _4, _5));

    this->m_updateDepthFrame_Connection    = m_depthCameraPtr->ConnectNewDepthFrame(boost::bind(
                                                                &GazeboYarpDepthCameraDriver::OnNewDepthFrame,
                                                                this, _1, _2, _3, _4, _5));

    std::cout << "this->m_updateImageFrame_Connection is " << this->m_updateImageFrame_Connection << std::endl;
    std::cout << "this->m_updateRGBPointCloud_Connection is " << this->m_updateRGBPointCloud_Connection << std::endl;
    std::cout << "this->m_updateDepthFrame_Connection is " << this->m_updateDepthFrame_Connection << std::endl;
    return true;
}

bool GazeboYarpDepthCameraDriver::close()
{
    // Copy & paste like a Monkey
    if (this->m_updateImageFrame_Connection.get())
    {
        m_imageCameraPtr->DisconnectNewImageFrame(this->m_updateImageFrame_Connection);
        this->m_updateImageFrame_Connection = gazebo::event::ConnectionPtr();
    }

    if (this->m_updateRGBPointCloud_Connection.get())
    {
        m_imageCameraPtr->DisconnectNewImageFrame(this->m_updateRGBPointCloud_Connection);
        this->m_updateRGBPointCloud_Connection = gazebo::event::ConnectionPtr();
    }

    if (this->m_updateDepthFrame_Connection.get())
    {
        m_imageCameraPtr->DisconnectNewImageFrame(this->m_updateDepthFrame_Connection);
        this->m_updateDepthFrame_Connection = gazebo::event::ConnectionPtr();
    }

    m_imageCameraSensorPtr = NULL;
    m_depthCameraSensorPtr = NULL;

    delete[] m_depthFrame_Buffer;
    m_depthFrame_Buffer = 0;
    m_depthFrame_BufferSize = 0;

    delete[] m_imageFrame_Buffer;
    m_imageFrame_Buffer = 0;
    m_imageFrame_BufferSize = 0;

    delete[] m_RGBPointCloud_Buffer;
    m_RGBPointCloud_Buffer = 0;
    m_RGBPointCloud_BufferSize = 0;

    return true;
}

/*
 * Get the image from the simulator and store it internally
 */
bool GazeboYarpDepthCameraDriver::OnNewImageFrame(const unsigned char *_image,
                          unsigned int _width, unsigned int _height,
                          unsigned int _depth, const std::string &_format)
{

    std::cout << "OnNewImageFrame: " <<
                    "\n\t_width is " << _width    <<
                    "\n\t_height is " << _height   <<
                    "\n\t_depth is " << _depth     <<
                    "\n\t_format is " << _format   << std::endl;

    m_dataMutex.wait();

    if(m_imageCameraSensorPtr->IsActive())
        memcpy(m_imageFrame_Buffer, m_imageCameraSensorPtr->GetImageData(), m_imageFrame_BufferSize);

    m_lastTimestamp.update(this->m_imageCameraSensorPtr->GetLastUpdateTime().Double());

    if (m_display_timestamp)
    {
        char txtbuf[1000];
            double time=yarp::os::Time::now()-start_time;
        sprintf(txtbuf,"%.3f",time);
        int len = strlen(txtbuf);
        if (len<20)
            print (m_imageFrame_Buffer,_width,_height,0,0,txtbuf, len);
    }

    m_dataMutex.post();
    return true;
}



/////////////////////////////////////////////////
void GazeboYarpDepthCameraDriver::OnNewRGBPointCloud(const float * /*_pcd*/,
                unsigned int _width, unsigned int _height,
                unsigned int _depth, const std::string &_format)
{
    std::cout << "OnNewRGBPointCloud: " <<
                "\n\t_width is " << _width    <<
                "\n\t_height is " << _height   <<
                "\n\t_depth is " << _depth     <<
                "\n\t_format is " << _format   << std::endl;
}

/////////////////////////////////////////////////
void GazeboYarpDepthCameraDriver::OnNewDepthFrame(const float * /*_image*/,
                                                        unsigned int _width,
                                                        unsigned int _height,
                                                        unsigned int _depth,
                                                        const std::string &_format)
{
  /*rendering::Camera::SaveFrame(_image, this->width,
    this->height, this->depth, this->format,
    "/tmp/depthCamera/me.jpg");  */

    std::cout << "OnNewDepthFrame: " <<
                "\n\t_width is " << _width    <<
                "\n\t_height is " << _height   <<
                "\n\t_depth is " << _depth     <<
                "\n\t_format is " << _format   << std::endl;

    // for depth camera
    if(m_depthCameraSensorPtr->IsActive())
        memcpy(m_depthFrame_Buffer, m_depthCameraPtr->GetDepthData(), m_depthFrame_BufferSize);   // change from sensor pointer to device pointer ... check carefully

}

// IFRAMEGRABBER IMAGE
bool GazeboYarpDepthCameraDriver::getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& _image)
{
     m_dataMutex.wait();
    _image.resize(m_width, m_height);

    unsigned char *pBuffer = _image.getRawImage();

    if (m_vertical_flip==true && m_horizontal_flip==false)
    {
    int r=0;
    int c=0;
        for (int c=0; c<m_width; c++)
        for (int r=0; r<m_height; r++)
        {
           unsigned char *pixel = _image.getPixelAddress(c,m_height-r-1);
           pixel[0] = *(m_imageFrame_Buffer + r*m_width*3+c*3+0);
           pixel[1] = *(m_imageFrame_Buffer + r*m_width*3+c*3+1);
           pixel[2] = *(m_imageFrame_Buffer + r*m_width*3+c*3+2);
        }
    }
    else if (m_vertical_flip==false && m_horizontal_flip==true)
    {
    int r=0;
    int c=0;
        for (int c=0; c<m_width; c++)
        for (int r=0; r<m_height; r++)
        {
           unsigned char *pixel = _image.getPixelAddress(m_width-c-1,r);
           pixel[0] = *(m_imageFrame_Buffer + r*m_width*3+c*3+0);
           pixel[1] = *(m_imageFrame_Buffer + r*m_width*3+c*3+1);
           pixel[2] = *(m_imageFrame_Buffer + r*m_width*3+c*3+2);
        }
    }
    else if (m_vertical_flip==true && m_horizontal_flip==true)
    {
        int r=0;
        int c=0;
        for (int c=0; c<m_width; c++)
        for (int r=0; r<m_height; r++)
        {
           unsigned char *pixel = _image.getPixelAddress(m_width-c-1,m_height-r-1);
           pixel[0] = *(m_imageFrame_Buffer + r*m_width*3+c*3+0);
           pixel[1] = *(m_imageFrame_Buffer + r*m_width*3+c*3+1);
           pixel[2] = *(m_imageFrame_Buffer + r*m_width*3+c*3+2);
        }
    }
    else
    {
        memcpy(pBuffer, m_imageFrame_Buffer, m_imageFrame_BufferSize);
    }

    if (m_display_time_box)
    {
        counter++;
        if (counter == 10) counter = 0; 
        for (int c=0+counter*30; c<30+counter*30; c++)
        for (int r=0; r<30; r++)
        {
           if (counter % 2 ==0)
           {unsigned char *pixel = _image.getPixelAddress(m_width-c-1,m_height-r-1);
           pixel[0] = 255;
           pixel[1] = 0;
           pixel[2] = 0;}
           else
           {unsigned char *pixel = _image.getPixelAddress(m_width-c-1,m_height-r-1);
           pixel[0] = 0;
           pixel[1] = 255;
           pixel[2] = 0;}
        }
    } 

    m_dataMutex.post();
    return true;
}

int GazeboYarpDepthCameraDriver::height() const
{
    return m_height;
}

int GazeboYarpDepthCameraDriver::width() const
{
    return m_width;
}

//PRECISELY TIMED
yarp::os::Stamp GazeboYarpDepthCameraDriver::getLastInputStamp()
{
    return m_lastTimestamp;
}

int GazeboYarpDepthCameraDriver::getRawBufferSize()
{
    return m_imageFrame_BufferSize;
}

