might need to publish 
robot_description_semantic from launchfile.


## robot getting stuck on itself

The problem is that when switching from IK control to the diff() taskspace controll is that it can push itself into configuration where it gets stuck or other dumb stuff. I tried adding joint repulse to keep them in middle of their paths, but that does not seem to do the trick.
I think ill have to try taskspace repulse on joint[3] to keep it away from pizza and also possibly the table.

# Robot not tracking correctly

There is a significant problem of the robot not being able to follow its desiered taskspace objective not only from the point of view of getting tangled. While the IK gives direct q, which can the jointspace controller stabilise quite well, I assume i cannot run it at high f and it quite often fails to converge - this sounds silly for usage as line following controller.
However the diff() taskspace controll is quite unpredictable. Sometimes it draws the line perfectly, sometimes the EE spins around its desiered location. I guess it depends on the robot configuration but there is not much i can do about it to controll it. 
Chat recomended the following:
- Task-Space Operational-Space Control (OSC) / Khatib Control
- Task-Space Inverse Kinematics (IK)
- Cartesian Impedance Control

 

 using sortof Cartesian Impedance Control - quite close to diff() controll, but this one has explicit dampening, - just spring dampener system, not mass, with quite high gains for the caresian part, the robot tracking is much better. Quite high dampening is needed (damp = 1.5)


 ## joint 6 just oscilating for some reason. Even heavyli overdampened, the 6 keeps oscilating. 
 So i decided to take the 6th joint out of the smart joint controller.



 ## the force measuring NEEDs a revolute joint apparently - cannot be fixed joint 
 