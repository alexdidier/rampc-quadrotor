# ROS Namespace Conventions

## Environment Variables

### ``ROS_NAMESPACE`` Environment Variable

All computers part of the ``D-FaLL-System`` should have the environment variable ``ROS_NAMESPACE`` set to the value ``dfall``.

You can check this from any terminal window with the command
```
echo $ROS_NAMESPACE
```

This environment vairable is set in the file:
```
dfall_ws/src/dfall_pkg/launch/Config.sh
```
The environemnt variable is set by the following line in that file:
```
export ROS_NAMESPACE='dfall'
```

### ``DEFAULT_{AGENT,COORD}_ID`` Environment Variables

The ``Config.sh`` also sets the environment variables for the default agent ID and the default coordinator ID, called ``DFALL_DEFAULT_AGENT_ID`` and ``DFALL_DEFAULT_COORD_ID``. These are set by the following line in the ``Config.sh`` file:
```
export DFALL_DEFAULT_AGENT_ID=$(cat /etc/dfall_default_agent_id)
export DFALL_DEFAULT_COORD_ID=$(cat /etc/dfall_default_coord_id)
```
This is the only place in the whole ``D-FaLL-System`` where the files ``/etc/dfall_default_{agent,coord}_id`` are referred to.

NOTE:
- These files should never be access from within any code,<br>
- To use these default ID's within any code, they should be added as parameters when launch the respective node,<br>
- The launch files is the only place where the default ID environment variables should be used.<br>



## Launching nodes

To be filled in