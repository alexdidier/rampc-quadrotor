//    Copyright (C) 2017, ETH Zurich, D-ITET, Angel Romero
//
//    This file is part of D-FaLL-System.
//    
//    D-FaLL-System is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//    
//    D-FaLL-System is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//    
//    You should have received a copy of the GNU General Public License
//    along with D-FaLL-System.  If not, see <http://www.gnu.org/licenses/>.
//    
//
//    ----------------------------------------------------------------------------------
//    DDDD        FFFFF        L     L           SSSS  Y   Y   SSSS  TTTTT  EEEEE  M   M
//    D   D       F      aaa   L     L          S       Y Y   S        T    E      MM MM
//    D   D  ---  FFFF  a   a  L     L     ---   SSS     Y     SSS     T    EEE    M M M
//    D   D       F     a  aa  L     L              S    Y        S    T    E      M   M
//    DDDD        F      aa a  LLLL  LLLL       SSSS     Y    SSSS     T    EEEEE  M   M
//
//
//    DESCRIPTION:
//    Look up table that connects the crazyflie names with their respective radio addresses
//
//    ----------------------------------------------------------------------------------


#include "channelLUT.h"


std::map<std::string, std::string> channel_LUT
{
    // {"CF1", "A12D2"},
    // {"CF2", "E341E"},
    // {"CF3", "4E21A"},
    {"cfOne", "0/76/2M/E7E7E7E701"},
    {"cfTwo", "0/69/2M"},
    {"cfThree", "0/72/2M"},
    {"cfFour", "0/99/2M"},
    {"PPS_CF01", "0/0/2M/E7E7E7E701"},
    {"PPS_CF02", "0/8/2M/E7E7E7E702"},
    {"PPS_CF03", "0/16/2M/E7E7E7E703"},
    {"PPS_CF04", "0/24/2M/E7E7E7E704"},
    {"PPS_CF05", "0/32/2M/E7E7E7E705"},
    {"PPS_CF06", "0/40/2M/E7E7E7E706"},
    {"PPS_CF07", "0/48/2M/E7E7E7E707"},
    {"PPS_CF08", "0/56/2M/E7E7E7E708"},
    {"PPS_CF09", "0/56/2M/E7E7E7E709"},
    {"PPS_CF10", "0/56/2M/E7E7E7E70A"},
};
