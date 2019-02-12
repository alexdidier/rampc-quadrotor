//    Copyright (C) 2019, ETH Zurich, D-ITET, Paul Beuchat
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
//    Constants that are used across the Default Controler
//    Service, its GUI, and the Flying Agent Client
//
//    ----------------------------------------------------------------------------------




// TO REQUEST MANOEUVRES

#define DEFAULT_CONTROLLER_REQUEST_TAKE_OFF    1
#define DEFAULT_CONTROLLER_REQUEST_LANDING     2


// TO IDENITFY THE STATE OF THE DEFAULT CONTROLLER

#define DEFAULT_CONTROLLER_STATE_UNKNOWN      -1
#define DEFAULT_CONTROLLER_STATE_STANDBY      99

// > The sequence of states for a TAKE-OFF manoeuvre
#define DEFAULT_CONTROLLER_STATE_TAKE_OFF_SPIN_MOTORS      11
#define DEFAULT_CONTROLLER_STATE_TAKE_OFF_MOVE_UP          12
#define DEFAULT_CONTROLLER_STATE_TAKE_OFF_GOTO_SETPOINT    13

// > The sequence of states for a LANDING manoeuvre
#define DEFAULT_CONTROLLER_STATE_LANDING_MOVE_DOWN        21
#define DEFAULT_CONTROLLER_STATE_LANDING_SPIN_MOTORS      22