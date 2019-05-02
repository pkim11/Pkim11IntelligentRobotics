#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <Planar2R.hpp>
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
	new_state.name = {"joint1", "joint2"};
	new_state.header.stamp = ros::Time::now();
	double diff = (ros::Time::now() - start).toSec();
	new_state.position = { M_PI * cos(diff), M_PI * sin(diff)};
	
	////
	// we need to create our object and grab their links
	// so we go to ../include/ROS_Planar2R.hpp to grab their links
	// and then we make the object, obj2r
	// n is the rosnode, or the handle of hte rosnode from above, line 20
	// we call the getParam function and grab the 
	// parameter "link_lengths/l1", which is hte link_lengths with l1 component of link_lengths
	// and then we store it in our variable, l1 
	// then we define the object and then call it to set the links to l1, l2
	double l1, l2;
	n.getParam("link_lengths/l1", l1);
	n.getParam("link_lengths/l2", l2);
	IRlibrary::Planar2R obj2r;
	obj2r.setLinks(l1, l2);


	// me trying out inverse kin
	//IRlibrary::Vec2 object;
	//object << l1/3., l2/2.;
	//obj2r.setXY(object);
	// works
	


	IRlibrary::Vec2 object;
	//object << l1/3., l2/2.;
	object << -2,1;
	IRlibrary::Vec2 increment;

	

	int incrementAmount = 10;
	obj2r.setXY(object);
	auto q = obj2r.getConfig();
	increment[0] = q[0]/incrementAmount;
	increment[1] = q[1]/incrementAmount;
	IRlibrary::Vec2 temp;
	
	//int i =0;
	//while (ros::ok())
	//{

	double arr[9][2] = {{-3.,2.},{-2.,1.},{1.,-2.},
						{-3.,2.},{-2.,1.},{1.,-2.},
						{-3.,2.},{-2.,1.},{1.,-2.}};
	for(int i=0; i<9 && ros::ok();i++){

		diff = (ros::Time::now() - start).toSec();

		object << arr[i][0], arr[i][1];
		IRlibrary::Planar2R test2r;
		test2r.setLinks(l1, l2);
		test2r.setXY(object);
		q = test2r.getConfig();
		
		new_state.position[0] = q[0];
		new_state.position[1] = q[1];
		
		configPub.publish(new_state);
		loop_rate.sleep();
		ros::Duration(1.0).sleep();
		//i++;

		////////
		// for simplicity, we will take trajectory of middle of workspace
		// first we get middle of radius
		// then we have some angle based on difference in time
		//double midRad = ((l1 + l2) + std::fabs(l1 - l2))/2.;
		//double ang = diff/2.;
		
		// next we create variables to hold the angles and set angles to the variable
		//IRlibrary::Vec2 xy;
		//xy << midRad*cos(ang), midRad*sin(ang);
		
		// now we can set the angles using setXY, from Planar2R.hpp
		// this automatically calls the inverseKinematics function from simple2r.cpp
		//obj2r.setXY(xy);
		//auto q = obj2r.getConfig();

/*
		if(diff < 10){

			//obj2r.setXY(object);
			//auto q = obj2r.getConfig();
			//new_state.header.stamp = ros::Time::now();
			//new_state.position[0] = q[0];
			//new_state.position[1] = q[1];
			new_state.position[0] = q[0];
			new_state.position[1] = q[1];
			//temp << q[0],q[1];
			//new_state.position[0] = increment[0] + increment[0];
			//new_state.position[1] = increment[1]+increment[1];
			//new_state.position[0] = q[0] * diff;
			//new_state.position[1] = q[1] * diff;
			//obj2r.setConfig(temp);
			configPub.publish(new_state);
			//ros::spinOnce();
			loop_rate.sleep();
		}
		*/

		//new_state.position[0] = q[0];
		//	new_state.position[1] = q[1];
			//temp << q[0],q[1];
			//new_state.position[0] = increment[0] + increment[0];
			//new_state.position[1] = increment[1]+increment[1];
			//new_state.position[0] = q[0] * diff;
			//new_state.position[1] = q[1] * diff;
			//obj2r.setConfig(temp);
			//configPub.publish(new_state);
			//ros::spinOnce();
			//loop_rate.sleep();
		

		////////
		/*
		new_state.header.stamp = ros::Time::now();
		new_state.position[0] = q[0];
		new_state.position[1] = q[1];
		configPub.publish(new_state);
		ros::spinOnce();
		loop_rate.sleep();
		/* Something important was published here */
		/* This was important as well, something spinning */
		/* Something related to sleep was here */
	}
	return 0;
}
