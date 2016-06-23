# Baxter Trac IK

Simple ROS server to query [trac_ik](https://bitbucket.org/traclabs/trac_ik/) package for Baxter.

## Tolerance feature
The package allows to specifiy a tolerance (Field `end_tolerance` of the IK request message). The chosen method makes several IK calculation step-by-step by increasing the tolerance until success or the maximum number of attempts have been reached (Field `num_steps` of the IK request message).

## How to use the IK service
### Download and compile the repositories

Make sure you haven't already downloaded the original [trac_ik repository](https://bitbucket.org/traclabs/trac_ik.git) or add an empty file `CATKIN_IGNORE` at the root of this package.

```
roscd
cd ../src
git clone https://github.com/baxter-flowers/trac_ik_baxter.git
git clone https://github.com/baxter-flowers/trac_ik.git
roscd
cd ..
catkin_make
```

### Start the service
`roslaunch trac_ik_baxter ik_server.launch`

### TRAC IK Request 

The default topics to send IK requests are:
```
/trac_ik_left
/trac_ik_right
```

You can pass a list of several poses stamped, the IK server hence returns a list of joint states, one for each input point, even if it's invalid. In that case the vector of booleans `isValid` is filled accordingly.

```
# Endpoint Pose(s) to request Inverse-Kinematics joint solutions for.
geometry_msgs/PoseStamped[] pose_stamp

# (optional) Joint Angle Seed(s) for IK solver.
# * specify a JointState seed for each pose_stamp, using name[] and position[]
sensor_msgs/JointState[] seed_angles

float32 end_tolerance
uint8 num_steps
```

If you do not need the IK server to increase tolerance in case of `IK not found` error, just leave `end_tolerance` and `num_steps` blank, although it then warns that tolerance is invalid the server will behave as the original TRAC IK package, performing only 1 attempt with tolerance = `1e-5`.

## TRAC IK Response 
```
# joints[i]      == joint angle solution for each pose_state[i]
sensor_msgs/JointState[] joints
float32[] accepted_tolerance
bool[] isValid
```
The `accepted_tolerance` field returns the tolerance for which we managed to find a solution (if any). Thus `accepted_tolerance<=end_tolerance`.
