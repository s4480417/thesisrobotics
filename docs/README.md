# Style Guide and Notation
## List of Contexts
* `arm` - Robotic Arms
* `cpv` - Computer Vision
* `mps` - Motion Planning System
* `usr` - User
* `utl` - Utility

## Robotic Arms
### Description
This context includes basic robot arm functionality, for both the left and right arm.
This includes joint angle control, cartesian position control, output
### Nodes
`rob/<name>/<l/r>_<node>`
### Messages
`rob_msgs/<MsgName>`
## Computer Vision
This context includes computer vision functionality, from the kinect and from openpose.
### Nodes
`cpv/<kin/opp>/<node>`
### Messages
`cpv_msgs/<MsgName>`
## Motion Planning System
### Nodes
`mps/<node>`
### Messages
`mps_msgs/<MsgName>`
## User
### Nodes
`usr/skel/<l/r>_<node>`
### Messages
`usr_msgs/<MsgName>`
## Utility
### Messages
`utl_msgs/<MsgName>`
