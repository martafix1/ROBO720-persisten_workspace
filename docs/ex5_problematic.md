The problem of repulsive joint filed is such that there is no reason for us to not allow the robot to reach its limits. 
With realism in mind, we just want the robot to reach is slowly, therefore to limit the approach speed.
This does not allow us to use just static fields, as it will necesearily overpower the P of our regulator sooner than the P forces the joint into its desired position. 
This leads me to the use of velocity dependend field.  

Assuming we just dont hardcap the velocity, which would require to go around the torque regulator, it is complicated. As it would be nice to have it not use the 
Kp and Kd constants of regulator to tune itself accordingly.

The tries before were not working as the discrete condition for their activation resulted in the jittering behaviour.

This leads me to an idea of viscose field, causing torque against direction of movement. By having it only as a vicose filed, it will be not able to truly stop the robot, it will just slow it down. 
For this to work however the field viscosity itself must be position dependent. 

The linear viscosity fields seems good, there is a bit of a step but it seems to reduce the velocity suficiently.

After fixing a mistake with the distance to "headed-to" limit, the viscose fields seems to work. 

Results:
This was run for the command being desired velocity
There seems to be no apparent difference between the behaviour for regulator omega = 20 and omega= 44 (44 comes from the w increase for lower joint)
The slowdown seems still bit abrupt. 

Switching to acceleration commands just results in actaully abrupt stop, as the command is not scaled by the 2* damp*w constant, this would need retuning

Trying out quadratic velocity dependence, keeping the viscosity itself linear. Tried the full quadratic (v^2 + v), trying just quadratic (v^2). The quadratic term needs to be corrected for direction. 