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


void callback(const sensor_msgs::JointStateConstPtr &msg) {	//pobranie wartości zadanych	
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

	double d1=0.2, a2=0.4; // domyślne wartości
	
	
	// dane z serwera parametrów

	if(!nh.getParam("/link_1", d1))
		{
			ROS_ERROR("Blad pobrania. Uzyto domyslne wartosci.");	
		}
	if(!nh.getParam("/link_2", a2))
		{
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
		

		q(0)=pozycjaZadana[0];
		q(1)=pozycjaZadana[1];
		q(2)=pozycjaZadana[2];

		solver.JntToCart(q,F);

		double x,y,z=0; // Współrzędne końcówki
		
		x = F.p.data[0];
		y = F.p.data[1]; 
		z = F.p.data[2];

		q(0)=pozycjaZadana[0]/2;
		q(1)=pozycjaZadana[1]/2;
		q(2)=pozycjaZadana[2]/2;
		solver.JntToCart(q,F);
	
		geometry_msgs::PoseStamped doWyslania;

		doWyslania.header.frame_id="base_link";

		doWyslania.pose.position.x = x; 
		doWyslania.pose.position.y = y;
		doWyslania.pose.position.z = z;

		doWyslania.pose.orientation.w = F.M.data[0];
		doWyslania.pose.orientation.x = F.M.data[4];
		doWyslania.pose.orientation.y = -F.M.data[1];
		doWyslania.pose.orientation.z = F.M.data[3];	

		pub.publish(doWyslania);
		rate.sleep();
	}
	return 0;
}
