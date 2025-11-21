might need to publish 
robot_description_semantic from launchfile.


## robot getting stuck on itself

The problem is that when switching from IK control to the diff() taskspace controll is that it can push itself into configuration where it gets stuck or other dumb stuff. I tried adding joint repulse to keep them in middle of their paths, but that does not seem to do the trick.
I think ill have to try taskspace repulse on joint[3] to keep it away from pizza and also possibly the table.