/*
 * Copyright (C) 2013-2015 Fondazione Istituto Italiano di Tecnologia RBCS & iCub Facility & ADVR
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_CONTROLBOARDDRIVERRANGE_HH
#define GAZEBOYARP_CONTROLBOARDDRIVERRANGE_HH

struct Range {
    Range() : min(0), max(0){}
    double min;
    double max;
};

#endif
