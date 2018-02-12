/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro, Alessandro Settimi and Marco Randazzo
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */


#include "CameraDriver.h"

#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/common.h>

#include <gazebo/sensors/CameraSensor.hh>
#include <yarp/os/Value.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace std;
using namespace yarp::dev;

const std::string YarpScopedName = "sensorScopedName";
double GazeboYarpCameraDriver::start_time =0;

void GazeboYarpCameraDriver::print (unsigned char* pixbuf, int pixbuf_w, int pixbuf_h, int x, int y, char* s, int size)
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

GazeboYarpCameraDriver::GazeboYarpCameraDriver()
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


GazeboYarpCameraDriver::~GazeboYarpCameraDriver()
{
}

//DEVICE DRIVER
bool GazeboYarpCameraDriver::open(yarp::os::Searchable& config)
{
    //Get gazebo pointers
    std::string sensorScopedName(config.find(YarpScopedName.c_str()).asString().c_str());

    // TODO get parent sensor, if it make any sense
    m_parentSensor = (gazebo::sensors::CameraSensor*)GazeboYarpPlugins::Handler::getHandler()->getSensor(sensorScopedName);
    if (!m_parentSensor) {
        yError() << "GazeboYarpCameraDriver Error: camera sensor was not found" ;
        return false;
    }

    if (config.check("vertical_flip")) m_vertical_flip =true;
    if (config.check("horizontal_flip")) m_horizontal_flip =true;
    if (config.check("display_timestamp")) m_display_timestamp =true;
    if (config.check("display_time_box")) m_display_time_box =true;

    m_camera = m_parentSensor->Camera();

    if(m_camera == NULL)
    {
        yError() << "GazeboYarpCameraDriver Error: camera pointer not valid";
        return false;
    }

    m_width  = m_camera->ImageWidth();
    m_height = m_camera->ImageHeight();

    m_bufferSize = 3*m_width*m_height;

    m_dataMutex.wait();
    m_imageBuffer = new unsigned char[3*m_width*m_height];
    memset(m_imageBuffer, 0x00, 3*m_width*m_height);
    m_lastTimestamp.update();
    m_dataMutex.post();

    //Connect the driver to the gazebo simulation
    this->m_updateConnection = m_camera->ConnectNewImageFrame(boost::bind(&GazeboYarpCameraDriver::captureImage, this, _1, _2, _3, _4, _5));
    return true;
}

bool GazeboYarpCameraDriver::close()
{
    if (this->m_updateConnection.get())
    {
        this->m_updateConnection.reset();
        m_parentSensor = NULL;
    }
    
    delete[] m_imageBuffer;
    m_imageBuffer = 0;

    return true;
}

/*
 * Get the image from the simulator and store it internally
 */
bool GazeboYarpCameraDriver::captureImage(const unsigned char *_image,
                          unsigned int _width, unsigned int _height,
                          unsigned int _depth, const std::string &_format)
{
    m_dataMutex.wait();

    if(m_parentSensor->IsActive())
        memcpy(m_imageBuffer, m_parentSensor->ImageData(), m_bufferSize);

    m_lastTimestamp.update(this->m_parentSensor->LastUpdateTime().Double());


    if (m_display_timestamp)
    {
	char txtbuf[1000];
    	double time=yarp::os::Time::now()-start_time;
	sprintf(txtbuf,"%.3f",time);
	int len = strlen(txtbuf);
   	if (len<20)
    	print (m_imageBuffer,_width,_height,0,0,txtbuf, len);
    }

    m_dataMutex.post();
    return true;
}


// IFRAMEGRABBER IMAGE
bool GazeboYarpCameraDriver::getImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>& _image)
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
           pixel[0] = *(m_imageBuffer+r*m_width*3+c*3+0);
           pixel[1] = *(m_imageBuffer+r*m_width*3+c*3+1);
           pixel[2] = *(m_imageBuffer+r*m_width*3+c*3+2);
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
           pixel[0] = *(m_imageBuffer+r*m_width*3+c*3+0);
           pixel[1] = *(m_imageBuffer+r*m_width*3+c*3+1);
           pixel[2] = *(m_imageBuffer+r*m_width*3+c*3+2);
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
           pixel[0] = *(m_imageBuffer+r*m_width*3+c*3+0);
           pixel[1] = *(m_imageBuffer+r*m_width*3+c*3+1);
           pixel[2] = *(m_imageBuffer+r*m_width*3+c*3+2);
        }
    }
    else
    {
        memcpy(pBuffer, m_imageBuffer, m_bufferSize);
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

int GazeboYarpCameraDriver::height() const
{
    return m_height;
}

int GazeboYarpCameraDriver::width() const
{
    return m_width;
}

//PRECISELY TIMED
yarp::os::Stamp GazeboYarpCameraDriver::getLastInputStamp()
{
    return m_lastTimestamp;
}

int GazeboYarpCameraDriver::getRawBufferSize()
{
    return m_bufferSize;
}

