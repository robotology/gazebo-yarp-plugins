/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT .
 */

#ifndef GAZEBOYARP_COMMON_H
#define GAZEBOYARP_COMMON_H

#include <string>
#include <cmath>
#include <vector>

#include <yarp/os/LogStream.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/Value.h>

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
     * \brief check if a string has a certaing ending
     * \param fullString the full string
     * \param ending the candidate ending
     * \return true if fullString ends with ending, false otherwise
     */
    bool hasEnding (std::string const &fullString, std::string const &ending);

    inline double convertDegreesToRadians(double degrees)
    {
        return degrees / 180.0 * pi;
    }

    inline double convertRadiansToDegrees(double radians)
    {
        return radians * 180.0 / pi;
    }

    inline bool hasEnding (std::string const &fullString, std::string const &ending)
    {
        if (fullString.length() >= ending.length()) {
            return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
        } else {
            return false;
        }
    }

    /**
     * \brief Get last part of string after separator
     * \param[in] fullString the full string
     * \param[in] separator separator string
     * \return lastPart the last part of the string, or the fullString if the seperator is not found
     */
    inline std::string lastPartOfStringAfterSeparator(std::string const &fullString, std::string const &separator)
    {
        auto pos = fullString.find_last_of(separator);
        if (pos == std::string::npos) {
            return fullString;
        } else {
            return fullString.substr(pos + separator.size() - 1);
        }
    }

    template<typename T>
    inline T readElementFromValue(yarp::os::Value& val);

    template<>
    inline double readElementFromValue<double>(yarp::os::Value& val)
    {
        return val.asFloat64();
    }

    template<>
    inline std::string readElementFromValue<std::string>(yarp::os::Value& val)
    {
        return val.asString();
    }

    /**
     * Get a vector from a parameter, using both the recommended style:
     * nameOfList (elem1 elem2 elem3)
     * or the deprecated (since YARP 3.10):
     * nameOfList elem1 elem2 elem3
     *
     *
     * \brief Get vector from YARP configuration
     * \return true if the parsing was successful, false otherwise
     */
    template <typename T>
    inline bool readVectorFromConfigFile(const yarp::os::Searchable& params, const std::string&listName, std::vector<T>& outputList)
    {
        bool vectorPopulated = false;
        outputList.resize(0);
        yarp::os::Value& val = params.find(listName);
        if (!val.isNull() && val.isList())
        {
            yarp::os::Bottle* listBot = val.asList();
            outputList.resize(listBot->size());

            for (size_t i=0; i < outputList.size(); i++)
            {
                outputList[i] = readElementFromValue<T>(listBot->get(i));
            }

            vectorPopulated = true;
        }
        else
        {
            // Try to interpreter the list via findGroup
            yarp::os::Bottle listBottleAndKey = params.findGroup(listName);
            if (!listBottleAndKey.isNull())
            {
                yWarning() << "Parameter " << listName << " should be a list, but its format is deprecated as parenthesis are missing."
                           << " Please add parentesis to the list, as documented in https://github.com/robotology/yarp/discussions/3092 .";

                outputList.resize(listBottleAndKey.size()-1);

                for (size_t i=0; i < outputList.size(); i++)
                {
                    outputList[i] = readElementFromValue<T>(listBottleAndKey.get(i+1));
                }
                vectorPopulated = true;
            }
        }
        return vectorPopulated;
    }

    /**
      * Convert a vector to a string for printing.
      */
    template <typename T>
    inline std::string vectorToString(std::vector<T>& outputList)
    {
        std::stringstream ss;
        for (size_t i = 0; i < outputList.size(); ++i) {
            ss << outputList[i];
            if (i != outputList.size() - 1)
            {
                ss << " ";
            }
        }
        return ss.str();
    }
}


#endif
