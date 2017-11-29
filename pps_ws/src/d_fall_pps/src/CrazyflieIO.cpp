//    Copyright (C) 2017, ETH Zurich, D-ITET, Cyrill Burgener, Marco Mueller, Philipp Friedli
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
//    This manages the Input/Output from the text file database that is used to store
//    and commincate an agent's zone between the coordinator node and the agent's node,
//    i.e., the details of:
//    > the linked ( agent ID , Crazyflie ) pair, where the Crazyflie is described by:
//      - the "name", which matches that used by the localisation system
//      - the "address", which is needed to establish a radio connection
//    > the hyper-rectangle area in which the agent is allowed to operate
//
//    ----------------------------------------------------------------------------------


#include "CrazyflieIO.h"
#include <stdlib.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <string>

#include "d_fall_pps/CrazyflieContext.h"
#include "d_fall_pps/CrazyflieEntry.h"
#include "d_fall_pps/CrazyflieDB.h"

using namespace std;

namespace d_fall_pps {

string escape(string input) {
    string escaped;
    for (string::const_iterator i = input.begin(), end = input.end(); i != end; ++i) {
        unsigned char c = *i;
        switch(c) {
            case '\\': escaped += "\\\\"; break;
            case '\r': escaped += "\\r"; break;
            case '\n': escaped += "\\n"; break;
            case '\t': escaped += "\\t"; break;
            case ',': escaped += "\\c"; break;
            default: escaped += c; break;
        }
    }
    return escaped;
}

string unescape(string input) {
    string unescaped;
    bool escapeSeq = false;
    for (string::const_iterator i = input.begin(), end = input.end(); i != end; ++i) {
        unsigned char c = *i;

        if(!escapeSeq) {
            if(c == '\\') {
                escapeSeq = true;
            } else {
                unescaped += c;
            }
        } else {
            switch(c) {
                case '\\': unescaped += "\\"; break;
                case 'r': unescaped += "\r"; break;
                case 'n': unescaped += "\n"; break;
                case 't': unescaped += "\t"; break;
                case 'c': unescaped += ","; break;
                default: ROS_ERROR_STREAM("illegal escape sequence: \"\\" << c << "\"");
            }
            escapeSeq = false;
        }
    }
    return unescaped;
}

vector<string> nextLine(istream& str) {
    vector<string> result;
    string line;
    getline(str,line);

    stringstream lineStream(line);
    string cell;

    while(getline(lineStream, cell, ',')) {
        result.push_back(cell);
    }

    //if there is a trailing comma with no data after it
    if (!lineStream && cell.empty()) {
        result.push_back("");
    }

    return result;
}

string getCrazyflieDBPath() {
    string packagePath = ros::package::getPath("d_fall_pps") + "/";
    string dbFile = packagePath + "param/Crazyflie.db";
    return dbFile;
}

void readCrazyflieDB(CrazyflieDB& db) {
    ifstream dbFile;
    dbFile.open(getCrazyflieDBPath());

    while(dbFile.peek() != EOF) {
        vector<string> dataRow = nextLine(dbFile);

        if(dataRow.size() == 0) {
        } else if(dataRow.size() != 10) {
            ROS_ERROR_STREAM("row in csv file has not the right amount of data fields, skipped");
        } else {

            CrazyflieEntry entry;
            entry.studentID = stoi(dataRow[0]);

            CrazyflieContext context;
            context.crazyflieName = unescape(dataRow[1]);
            context.crazyflieAddress = unescape(dataRow[2]);

            AreaBounds area;
            area.crazyfly_zone_index = stof(dataRow[3]);
            area.xmin = stof(dataRow[4]);
            area.ymin = stof(dataRow[5]);
            area.zmin = stof(dataRow[6]);

            area.xmax = stof(dataRow[7]);
            area.ymax = stof(dataRow[8]);
            area.zmax = stof(dataRow[9]);

            context.localArea = area;
            entry.crazyflieContext = context;
            db.crazyflieEntries.push_back(entry);
        }
    }
}

void writeCrazyflieDB(CrazyflieDB& db) {
    ofstream dbFile;
    dbFile.open(getCrazyflieDBPath());

    for(int i = 0; i < db.crazyflieEntries.size(); ++i) {
    	CrazyflieEntry entry = db.crazyflieEntries[i];
    	CrazyflieContext context = entry.crazyflieContext;

    	dbFile << entry.studentID << ',';
    	dbFile << escape(context.crazyflieName) << ',';
    	dbFile << escape(context.crazyflieAddress) << ',';

    	AreaBounds area = context.localArea;

        dbFile << area.crazyfly_zone_index << ',';

        dbFile << area.xmin << ',';
    	dbFile << area.ymin << ',';
    	dbFile << area.zmin << ',';

    	dbFile << area.xmax << ',';
    	dbFile << area.ymax << ',';
    	dbFile << area.zmax;

    	dbFile << '\n';
    }

    dbFile.close();
}

}
