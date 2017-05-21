#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include <signal.h>
#include <stdio.h>
#include <iostream>
#include <math.h>


double pozycjaZadana[3];
double const PI = M_PI;
bool joint_state_aktywny=false;

void callback(const sensor_msgs::JointStateConstPtr &msg) {	//pobranie wartości zadanych
	joint_state_aktywny=true;	
	for (int i = 0; i < 3; i++) {
		pozycjaZadana[i]=msg->position[i];
	}
}

int main(int argc, char **argv) {
	
	ros::init(argc,argv,"DKIN");	//inicjalizacja ROSa
	ros::NodeHandle nh;
	ros::Publisher pub=nh.advertise<geometry_msgs::PoseStamped>("/PoseStampedNONKDL", 1000);
	ros::Subscriber sub=nh.subscribe<sensor_msgs::JointState>("/joint_states", 1000, boost::bind(callback,_1));
	ros::Rate rate(30);

	double d1=1.0, a2=0.5; // domyślne wartości
	double joint2_lower=-3.915, joint2_upper=0.775, joint3_lower=-2.365, joint3_upper=2.365;  
	
	// dane z serwera parametrów
	if(!nh.getParam("/link1", d1)) {
			ROS_ERROR("Blad pobrania. Uzyto domyslne wartosci.");	
		}
	if(!nh.getParam("/arm2", a2)) {
			ROS_ERROR("Blad pobrania. Uzyto domyslne wartosci.");	
		}
	if(!nh.getParam("/joint2_lower",joint2_lower)) {
			ROS_ERROR("Blad pobrania. Uzyto domyslne wartosci.");	
		}
	if(!nh.getParam("/joint2_upper",joint2_upper)) {
			ROS_ERROR("Blad pobrania. Uzyto domyslne wartosci.");	
		}
	if(!nh.getParam("/joint3_lower",joint3_lower)) {
			ROS_ERROR("Blad pobrania. Uzyto domyslne wartosci.");
		}
	if(!nh.getParam("/joint3_upper",joint3_upper)) {
			ROS_ERROR("Blad pobrania. Uzyto domyslne wartosci.");
		}

		
	while(ros::ok()) {
		ros::spinOnce(); // Pobranie informacji od Joint_State_Publishera
		
		if(!joint_state_aktywny) // czy dane odebrane od joint_state_publishera
			continue;
		
		// ograniczenia kinematyczne manipulatora
		if(pozycjaZadana[1] > joint2_upper || pozycjaZadana[1] < joint2_lower) {
			ROS_ERROR("Wyznaczenie polozenia nie jest mozliwe.");
			continue;
		}
		
		if(pozycjaZadana[2] > joint3_upper || pozycjaZadana[2] < joint3_lower) {
			ROS_ERROR("Wyznaczenie polozenia nie jest mozliwe.");
			continue;
		}

		double t1=pozycjaZadana[0];
		double t2=pozycjaZadana[1];
		double t3=pozycjaZadana[2];
				
		geometry_msgs::PoseStamped doWyslania;

		doWyslania.header.frame_id="base_link";

		doWyslania.pose.position.x = a2*cos(t1)*cos(t2); 
		doWyslania.pose.position.y = a2*cos(t2)*sin(t1);
		doWyslania.pose.position.z = d1-a2*sin(t2);

		doWyslania.pose.orientation.w = cos(t1/2)*cos((t2+t3)/2);
		doWyslania.pose.orientation.x = -sin(t1/2)*sin((t2+t3)/2);
		doWyslania.pose.orientation.y = cos(t1/2)*sin((t2+t3)/2);
		doWyslania.pose.orientation.z = sin(t1/2)*cos((t2+t3)/2);	

		pub.publish(doWyslania);
		rate.sleep();
	}
	return 0;
}
