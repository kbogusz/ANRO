#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"


using namespace std;


double t[3];
double pos[3];
double a2,a3,d1; //default 0.4 0.2 0.2


double init_t[]={0, -M_PI/4, M_PI/4};

void inverse_kin(double pos[3], double t[3])
{
    bool exit=false;
   
    
    double x,y,z,e,f,c,A,B,G,cosB,cosG;
    x=pos[0];
    y=pos[1];
    z=pos[2];
    
    double eps=0.01;
    
    double a22=pow(a2,2);
    double a32=pow(a3,2);
    
    double x0=0;
    double y0=d1;
    double x1=sqrt(pow(x,2)+pow(y,2));
    double y1=d1;
    double x2=x1;
    double y2=z;
    
    c=sqrt(pow(x2-x0,2)+pow(y2-y0,2));
    e=sqrt(pow(x1-x0,2)+pow(y1-y0,2));
    f=y2-y1;
    
    A=atan(f/e);
    
    cosB=(a32-a22-pow(c,2))/(-2*a2*c);
    B=acos(cosB);
    
    cosG=(pow(c,2)-a22-a32)/(-2*a2*a3);
    G=acos(cosG);
           
	t[0]=atan2(y,x);
	t[1]=-A-B;
	t[2]=M_PI-G;
	
	if(t[1]<-M_PI/2-eps) t[1]=-M_PI/2;
	if(t[1]>0+eps) t[1]=0;

		
}

void callback(const geometry_msgs::PoseStamped & msg)
{
	pos[0]=msg.pose.position.x;
	pos[1]=msg.pose.position.y;
	pos[2]=msg.pose.position.z;

	inverse_kin(pos,t);
} 

void fillmsg(sensor_msgs::JointState &msg, double pos[])
{
    msg.header.stamp = ros::Time::now(); 
    msg.name.push_back("base_to_link_1");
    msg.name.push_back("link_1_to_link_2");
    msg.name.push_back("link_2_to_link_3");
    
	msg.position.push_back(pos[0]);
	msg.position.push_back(pos[1]);
	msg.position.push_back(pos[2]);

}



int main(int argc, char **argv)
{	
    //inicjacja ros
	ros::init(argc, argv, "ikin");
	ros::NodeHandle n;
	
    //utworzenie pub i sub
	ros::Publisher ikin_pub=n.advertise<sensor_msgs::JointState>("joint_states",1000);
	ros::Subscriber ikin_sub=n.subscribe("oint_pos",1000,callback);	
    
    //pobranie parametrów
	if(!n.getParam("a2", a2))
		{
			ROS_ERROR("Błąd pobrania. Użyte domyślne wartości.");	
		}
	if(!n.getParam("a3", a3))
		{
			ROS_ERROR("Błąd pobrania. Użyte domyślne wartości.");	
		}

	if(!n.getParam("d1", d1))
		{
			ROS_ERROR("Błąd pobrania. Użyte domyślne wartości.");	
		}

    
    //ROS_DEBUG_STREAM(a2<<" "<<a3<<" "<<d1);
    
	ros::Rate loop_rate(100);
	
	for(int i=0; i<3; i++)
	{
		t[i]=0;
	}
		
	while(ros::ok())
	{	

   		sensor_msgs::JointState msg;
    	fillmsg(msg,t);
  		ikin_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}

return 0;
}
