# 3dPrinter

This is a software developed part of a thesis project.

This software helps plan the process of robotic additive manufacturing using liquid metal.

<<<<<<< HEAD
The software takes a 3D CAD file as an input and plans the path for a 6-axis industrial
robotic manipulator to additively manufacture the input model.
Once the path is generated the software also allows the user to simulated the process
before exporting the generated robot path to a physical robot to start 3D printing.

Once the user is happy with the path, it is sent to the robot for additive manufacturing.
In this project the software uses a robot, with a weld torch as its tool to deposit the liquid metal,
and a method known as drop-by-drop weld deposition is used.  
=======
The software takes a 3D CAD file as an input and plans the path for a 6-axis industrial robotic manipulator to additively manufacture the input model. Once the path is generated the software also allows the user to simulated the process before exporting the generated robot path to a physical robot to start 3D printing. 

Once the user is happy with the path, it is sent to the robot for additive manufacturing. In this project the software uses a robot, with a weld torch as its tool to deposit the liquid metal, and a method known as drop-by-drop weld deposition is used.  
>>>>>>> 2f810eb2bbf98e22229a7721368d94d00e68ef63

This software depends on several dependancies such as:

Boost
Bullet Physics
Ogre
Eigen
Assimp
<<<<<<< HEAD
Lua
RobSim

RobSim is a custom library built at UOW and is not made avaliable in this repo due to proprietary issues.
This is the main library used to build this metal 3D printing software.
=======
Lua 
RobSim

RobSim is a custom library built at UOW and is not made avaliable in this repo due to proprietary issues. This is the main library used to build this metal 3D printing software. 
>>>>>>> 2f810eb2bbf98e22229a7721368d94d00e68ef63

Future Work:

~ Setup a control strategy to monitor and carefully change the weld parameters.
~ Change the software to allow for user input of weld settings to determine the expected weld droplet geometry.
~ The control strategy, and the user input of the weld system must be able to control a consistent droplet geometry.
<<<<<<< HEAD
~ Ability to move & angle the torch to have the stick out wire perpendicular to the droplet surface to help with a consistent build of layers.
~ A function to Interpolate the path's target loaction, needs to be built to print models with complex features.
=======
~ Ability to move & angle the torch to have the stick out wire perpendicular to the droplet surface to help with a consistent build of       layers.
~ A function to Interpolate the path's target loaction, needs to be built to print models with complex features. 
>>>>>>> 2f810eb2bbf98e22229a7721368d94d00e68ef63
