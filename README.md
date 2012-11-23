robot-parser
============

A basic parser/writer for robot model (from vrml to urdf)


Compilation
===========

mkdir build;
cd build;
cmake ..
make -s 

the executable is stored in bin


Execution
=========

Run the program from the bin folder.
It will read the following files from your folder ${path-of-robot-parser}/data/your_robot_name

VRMLmail.wrl,    	(mandatory) that corresponds to the desciption of the multibody in the openhrp vrml format.
jointLimits.dat  	(optional) defines additional joint limits (velocity, torque)
bodynametourdf.txt	(optional) defines associations between body names in the vrml (input) and body names in the urdf (output)
bodynametovrml.txt	(optional) defines associations between body names in the vrml (input) and body names in the vrml (output)
bodynametovrml_BV.txt	(optional) defines the bodies that will have a Bounding volume


