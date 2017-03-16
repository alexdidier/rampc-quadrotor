///////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) OMG Plc 2009.
// All rights reserved.  This software is protected by copyright
// law and international treaties.  No part of this software / document
// may be reproduced or distributed in any form or by any means,
// whether transiently or incidentally to some other use of this software,
// without the written permission of the copyright owner.
//
///////////////////////////////////////////////////////////////////////////////

#include "ros/ros.h"

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "ViconDataStreamSDK");
    ROS_INFO_STREAM("Okili dokili");
}
