#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include <signal.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>

using namespace KDL;

double pozycjaZadana[3];
bool joint_state_aktywny=false;

void callback(const sensor_msgs::JointStateConstPtr &msg) {	//pobranie wartości zadanych
	joint_state_aktywny=true;	
	for (int i = 0; i < 3; i++) {
		pozycjaZadana[i]=msg->position[i];
	}
}

int main(int argc, char **argv) {
	
	ros::init(argc,argv,"KDLDKIN");	//inicjalizacja ROSa
	ros::NodeHandle nh;
	ros::Publisher pub=nh.advertise<geometry_msgs::PoseStamped>("/PoseStampedKDL", 1000);
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

	KDL::Chain chain; // Stworzenie łańcucha kinematycznego
	
	chain.addSegment(Segment(Joint(Joint::None),Frame(Frame::DH(0, 0, 0, 0))));
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(0, 0, d1, 0))));
	chain.addSegment(Segment(Joint(Joint::None),Frame(Frame::DH(0, -M_PI/2, 0, 0))));
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(a2, 0, 0, 0))));
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(0, 0, 0, 0))));

	ChainFkSolverPos_recursive solver(chain);
	JntArray q(chain.getNrOfJoints());
	Frame F;		
	
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
	
		geometry_msgs::PoseStamped doWyslania;
		doWyslania.header.frame_id="base_link";
		
		q(0)=pozycjaZadana[0];
		q(1)=pozycjaZadana[1];
		q(2)=pozycjaZadana[2];
		solver.JntToCart(q,F);

		doWyslania.pose.position.x = F.p.data[0]; 
		doWyslania.pose.position.y = F.p.data[1];
		doWyslania.pose.position.z = F.p.data[2];
		
		q(0)=pozycjaZadana[0]/2;
		q(1)=pozycjaZadana[1]/2;
		q(2)=pozycjaZadana[2]/2;
		solver.JntToCart(q,F);

		doWyslania.pose.orientation.w = F.M.data[0];
		doWyslania.pose.orientation.x = F.M.data[4];
		doWyslania.pose.orientation.y = -F.M.data[1];
		doWyslania.pose.orientation.z = F.M.data[3];	

		pub.publish(doWyslania);
		rate.sleep();
	}
	return 0;
}
