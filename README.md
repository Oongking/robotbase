# robotbase

<p align="center">
  <img src="./docs/gazebo_envi.jpg" width=80% height=80%>
</p>

Need to add models path into .bashrc file
```
export GAZEBO_RESOURCE_PATH=~/(workspace)/src/robotbase/models:${GAZEBO_RESOURCE_PATH}
export GAZEBO_MODEL_PATH=~/(workspace)/src/robotbase/models:${GAZEBO_MODEL_PATH}
```

To open the gazebo simulation with joystick
```
roslaunch robotbase sim_map.launch 
```
Can use joystick to control the robot 

  | Button | Description |
  | --- | --- |
  | `Up` | Robot go **+X** or go **Forward**|
  | `Down` | Robot go **-X** or go **Backward** |
  | `Right` | Robot go **-Y** or go **Right** |
  | `Left` | Robot go **+Y** or go **Left** |
  | `L1` | Robot heading go **+Z** |
  | `R1` | Robot heading go **-Z** |

  | Analog | Description |
  | --- | --- |
  | `X left analog` | Robot go **+-Y** |
  | `Y left analog` | Robot go **+-X** |
  | `X right analog` | Robot heading go **+-Z** |
