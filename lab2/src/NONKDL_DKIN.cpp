#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include <signal.h>
#include <stdio.h>
#include <iostream>
#include <math.h>


double pozycjaZadana[3];
double x = 0, y = 0, z = 0;

// Funkcja do pobrania wartości zadanych
void callback(const sensor_msgs::JointStateConstPtr &msg) {
	for (int i = 0; i < 3; i++) {
		pozycjaZadana[i] = msg->position[i];
	}
}

int main(int argc, char **argv) {
	
	// Inicjalizacja ros-a
	ros::init(argc,argv,"DKIN");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/PoseStampedNONKDL", 1000
);
	ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 1000, boost::bind(callback,_1));
	ros::Rate rate(30);
	double d1 = 0.2, a2 = 0.4;
	//double d1, a2;
	std::string parametry;
	double qw, qx, qy, qz;

	if(!nh.getParam("/link_1", d1))
		{
			ROS_ERROR("Błąd pobrania. Użyte domyślne wartości.");	
		}
	if(!nh.getParam("/link_2", a2))
		{
			ROS_ERROR("Błąd pobrania. Użyte domyślne wartości.");	
		}
	if(nh.getParam("/robot_description",parametry)) {
			std::cout << "mam parametry" << std::endl << parametry;		
	}


	
	while(ros::ok()) {
		ros::spinOnce(); // Pobranie informacji od Joint_State_Publishera
		double t1 = pozycjaZadana[0];
		double t2 = pozycjaZadana[1];
		double t3 = pozycjaZadana[2];
		
		
		// Obliczanie parametrów do wysłania
		x = a2*cos(t1)*cos(t2);
		y = a2*cos(t2)*sin(t1);
		z = d1 - a2*sin(t2);
		// Ewentualnie dodać ograniczenia i logowanie
	

	//macierz przeksztalcenia jednorodnego
	double m00 = cos(t1)*cos(t2)*cos(t3) - cos(t1)*sin(t2)*sin(t3);
	double m10 = cos(t2)*cos(t3)*sin(t1) - sin(t1)*sin(t2)*sin(t3);
	double m20 = -cos(t2)*sin(t3) - cos(t3)*sin(t2);
	double m30 = 0;
	double m01 = -cos(t1)*cos(t2)*sin(t3) - cos(t1)*cos(t3)*sin(t2);
	double m11 = -cos(t2)*sin(t1)*sin(t3) - cos(t3)*sin(t1)*sin(t2);
	double m21 = sin(t2)*sin(t3) - cos(t2)*cos(t3);
	double m31 = 0;
	double m02 = -sin(t1);
	double m12 = cos(t1);
	double m22 = 0;
	double m32 = 0;
	double m03 = a2*cos(t1)*cos(t2);
	double m13 = a2*cos(t2)*sin(t1);
	double m23 = d1 - a2*sin(t2);
	double m33 = 1;

	//trace do obliczania kwaternionu
	double tr = m00 + m11 + m22;

	//obliczanie kwaternionu
	if (tr > 0) { 
	  double S = sqrt(tr+1.0) * 2; // S=4*qw 
	  qw = 0.25 * S;
	  qx = (m21 - m12) / S;
	  qy = (m02 - m20) / S; 
	  qz = (m10 - m01) / S; 
	} else if ((m00 > m11)&(m00 > m22)) {
	 
	  double S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx 
	  qw = (m21 - m12) / S;
	  qx = 0.25 * S;
	  qy = (m01 + m10) / S; 
	  qz = (m02 + m20) / S; 
	}else if (m11 > m22) {
	 
	  double S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
	  qw = (m02 - m20) / S;
	  qx = (m01 + m10) / S; 
	  qy = 0.25 * S;
	  qz = (m12 + m21) / S; 
	}else { 
	  double S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
	  qw = (m10 - m01) / S;
	  qx = (m02 + m20) / S;
	  qy = (m12 + m21) / S;
	  qz = 0.25 * S;
	}

	geometry_msgs::PoseStamped doWyslania;
	doWyslania.header.frame_id = "base_link";
	
	doWyslania.pose.position.x = x; 
	doWyslania.pose.position.y = y;
	doWyslania.pose.position.z = z+0.1;
	
	doWyslania.pose.orientation.w = qw;
	doWyslania.pose.orientation.x = qx;
	doWyslania.pose.orientation.y = qy;
	doWyslania.pose.orientation.z = qz;
	
	pub.publish(doWyslania);
	rate.sleep();
	}

	return 0;
}
