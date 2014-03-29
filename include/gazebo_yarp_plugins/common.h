/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: Silvio Traversaro, Enrico Mingo, Alessio Rocchi, Mirko Ferrati and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef GAZEBOYARP_COMMON_H
#define GAZEBOYARP_COMMON_H

namespace GazeboYarpPlugins {
        
    double const pi = 3.1415926535897932384626433;
    
    /**
     * \brief convert from degrees to radians
     * \param degrees angle in degrees
     * \return the angle converted in radians
     */
    double convertDegreesToRadians(double degrees);
    
    /**
     * \brief convert from radians to degrees
     * \param radians angle in radians
     * \return the angle converted in degrees
     */
    double convertRadiansToDegrees(double radians);
    
    
    inline double convertDegreesToRadians(double degrees) { return degrees / 180.0 * pi; }
    
    inline double convertRadiansToDegrees(double radians) { return radians * 180.0 / pi; }
}


#endif
