/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: Silvio Traversaro, Enrico Mingo, Alessio Rocchi, Mirko Ferrati and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef GAZEBOYARP_COMMON_H
#define GAZEBOYARP_COMMON_H

#include <string>
#include <cmath>

namespace GazeboYarpPlugins {

    ///< Seconds to wait for an answer when trying to connect to the yarpserver
    const double yarpNetworkInitializationTimeout = 10.0;

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

    /**
     * \brief convert a degrees-based gain to a radians-based gain
     * \param gainsWrtDegrees degrees-based gain
     * \return the gain converted in radians-based units
     */
    double convertDegreesGainsToRadiansGains(double gainsWrtDegrees);

    /**
     * \brief convert a radians-based gain to a degrees-based gain
     * \param gainsWrtRadians radians-based gain
     * \return the gain converted in degrees-based units
     */
    double convertRadiansGainsToDegreesGains(double gainsWrtRadians);

    /**
     * \brief check if a string has a certaing ending
     * \param fullString the full string
     * \param ending the candidate ending
     * \return true if fullString ends with ending, false otherwise
     */
    bool hasEnding (std::string const &fullString, std::string const &ending);


    inline double convertDegreesToRadians(double degrees) { return degrees / 180.0 * pi; }

    inline double convertRadiansToDegrees(double radians) { return radians * 180.0 / pi; }

    inline bool hasEnding (std::string const &fullString, std::string const &ending)
    {
        if (fullString.length() >= ending.length()) {
            return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
        } else {
            return false;
        }
    }

    inline double convertDegreesGainsToRadiansGains(double gainsWrtDegrees)
    {
        return gainsWrtDegrees * 180.0 / pi;
    }

    inline double convertRadiansGainsToDegreesGains(double gainsWrtRadians)
    {
        return gainsWrtRadians / 180.0 * pi;
    }

}


#endif
