#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <Planar2R.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <Spatial3R.hpp>

	/* Write your code her for publishing to /pubJointStates topic
	** The message type is sensor_msgs/JointState 
	** The name field should be an array of names of all four joints
	** The header.stamp field should be ros::Time::now() 
	** The position field should be an array of double values
	** Keep filling the values inside the while(ros::ok()) loop
	** Elapsed time can be calculated as:
	** ros::Time start = ros::Time::now();
	** double diff = (ros::Time::now() - start).toSec();
	** Make the values sinusodial depending on variable diff or anything you like
	** Publish the msg 
	** The lines to be changed or added are marked*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "genConfig");
	ros::NodeHandle n;
	ros::Rate loop_rate(30);

	ros::Duration(0.01).sleep();
	ros::Publisher configPub;
	configPub = n.advertise <sensor_msgs::JointState> ("/pubJointStates", 5); /* Fix this line. Do NOT change "/pubJointStates" */
	ros::Time start = ros::Time::now();
	sensor_msgs::JointState new_state;
	new_state.name = {"joint1", "joint2","joint3"};
	new_state.header.stamp = ros::Time::now();
	double diff = (ros::Time::now() - start).toSec();
	
	
	////
	// here we have to set new_state.position to some function of xyz
//	new_state.position = { M_PI * cos(diff), M_PI * sin(diff), 0 	};
	new_state.position = { 0, 0, 0 	};
	// we need to create our object and grab their links
	// so we go to ../include/ROS_Planar2R.hpp to grab their links
	// and then we make the object, obj2r
	// n is the rosnode, or the handle of hte rosnode from above, line 20
	// we call the getParam function and grab the 
	// parameter "link_lengths/l1", which is hte link_lengths with l1 component of link_lengths
	// and then we store it in our variable, l1 
	// then we define the object and then call it to set the links to l1, l2
	double l1, l2, l3;
	n.getParam("link_lengths/l1", l1);
	n.getParam("link_lengths/l2", l2);
	n.getParam("link_lengths/l3", l3);

	// objectList is the list of object positions
	// object is the current object that will be picked up/placed
	// placementLine is the line where the object will be placed
	double objectList[9][3] = {{3.,3,3.},{3.,3.,3.},{3.,3.,3.},
						{3.,3,3.},{3.,3.,3.},{3.,3.,3.},
						{3.,3,3.},{3.,3.,3.},{3.,3.,3.}};
	IRlibrary::Vec3 object;
	IRlibrary::Vec3 placementLine;

	while (ros::ok()){
	for(int i=0; i<9 && ros::ok();i++){
		//diff = (ros::Time::now() - start).toSec();

		//create the robot and set links
		IRlibrary::Spatial3R obj3r;
		obj3r.setLinks(l1, l2, l3);
		auto q = obj3r.getConfig();
		
		//grab an object
		object << objectList[i][0], objectList[i][1], objectList[i][2];
		object << 0,0,0;
		object << 5,0, 3;
		obj3r.setX(object);
		q = obj3r.getConfig();
		new_state.position[0] = q[0];
		new_state.position[1] = q[1];
		new_state.position[2] = q[2];
		configPub.publish(new_state); ros::Duration(1.0).sleep();

		// putting objects on line, random x, static y, static z
//*		double randomNumber;
		double randomNumber = (double)(rand()%9)-4;
		placementLine << 1,1,1;
		obj3r.setX(placementLine);
		q = obj3r.getConfig();
		new_state.position[0] = q[0];
		new_state.position[1] = q[1];
		new_state.position[2] = q[2];
		configPub.publish(new_state); ros::Duration(1.0).sleep();		
/*/
		//increment[0] = q[0]/incrementAmount;
		//increment[1] = q[1]/incrementAmount;
		/*
		for(int j=0;j<incrementAmount;j++){
			new_state.position[0] = new_state.position[0] + increment[0];
			new_state.position[1] = new_state.position[1] + increment[1];
			configPub.publish(new_state);
			ros::Duration(.02).sleep();
		}*/
	}
	}
	return 0;
}
