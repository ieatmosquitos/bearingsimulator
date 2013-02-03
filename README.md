bearingsimulator
================

A small simulator for a bearing only sensor equipped robot.

It outputs a file suitable to be fed to the other project "bearingslam"

Needed libraries are:
• SFML
• Eigen


COMMANDS:
• left mouse click -- add a landmark
• ESC key -- quit
• Q/W key -- increase/decrease sensor range
• UP/DOWN ARROW -- increase/decrease "speed"
• LEFT/RIGHT ARROW -- increase/decrease clockwise/counterclockwise "rotation speed"
• ENTER -- stops the robot and puts "speed" and "rotation speed" to zero.

NOTE:
• the gray robot represents the robot guess about its position
• the program outputs 2 files: truth.trj represents the "real" data, while noised.trj has noise on the odometry and on the perceptions.
• both files have the same format:
       every line is a data acquisition in the form "x y theta perception1 ... perceptionN"
