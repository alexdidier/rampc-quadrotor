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
//    Constants that are used across the Picker Controler Service and GUI
//
//    ----------------------------------------------------------------------------------





// TO IDENITFY THE STATE OF THE PICKER

#define PICKER_STATE_UNKNOWN      -1
#define PICKER_STATE_STANDBY       0
#define PICKER_STATE_GOTO_START    1
#define PICKER_STATE_ATTACH        2
#define PICKER_STATE_LIFT_UP       3
#define PICKER_STATE_GOTO_END      4
#define PICKER_STATE_PUT_DOWN      5
#define PICKER_STATE_SQUAT         6
#define PICKER_STATE_JUMP          7



// DEFAULT {x,y,z,yaw,mass}

#define PICKER_DEFAULT_X               0
#define PICKER_DEFAULT_Y               0
#define PICKER_DEFAULT_Z               0.4
#define PICKER_DEFAULT_YAW_DEGREES     0
#define PICKER_DEFAULT_MASS_GRAMS     30