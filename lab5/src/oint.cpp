#include "ros/ros.h"
#include "lab5/oint_point.h"
#include "lab5/oint_trajectory.h"
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

#define RATE 50
double elipse_step = M_PI/30;

double pos[3],prev_pos[3],fig_pos[3];
double czas;
ros::Publisher oint_pub;
ros::Publisher trajectory_pub;
double sqr[4][3] = {{0.4,0.2,0.2},{0.4,-0.2,0.2},{0.4,-0.2,0.6},{0.4,0.2,0.6}};
int point_number=1;
double theta=0;
double tbg;
bool follow_square=false;
bool follow_elipse=false;
nav_msgs::Path path;
double last_path_time=0;

double elipse_a=0.3;
double elipse_b=0.2;

bool is_out_of_range(double x, double y, double z)
{
    if (z<0) return true;
    if (z>0.8)  return true;
    if (x>0.6 || x<-0.6) return true;
    if (y>0.6 || y<-0.6) return true;
    if ((x>0.4 || x<-0.4) && z<0.2) return true;
    if ((z>0.4 || z<-0.4) && z<0.2) return true;
    if (sqrt(x*x+y*y)>0.6) return true;
    double x0=0;
    double y0=0.2;
    double x1=sqrt(pow(x,2)+pow(y,2));
    double y1=0.2;
    double x2=x1;
    double y2=z;
    
    if(sqrt(pow(x2-x0,2)+pow(y2-y0,2))>0.6) return true;
    
    
    return false; 
    
}

void gen_elipse_point(double &x, double &y, double &z)
{
   x=elipse_a*cos(theta);
   y=0.4;
   z=elipse_b*sin(theta)+0.4;
   theta+=elipse_step;
   if(theta>2*M_PI) theta=0;
   
}
void fillmsg(geometry_msgs::PoseStamped &msg, double pos[])
{

    msg.header.stamp = ros::Time::now(); 
    msg.header.frame_id ="base_link";
    msg.pose.orientation.x=0;
	msg.pose.orientation.y=0;
	msg.pose.orientation.z=0;
	msg.pose.orientation.w=1;
	msg.pose.position.x=pos[0];
	msg.pose.position.y=pos[1];
	msg.pose.position.z=pos[2];

}

void fillpathmsg(nav_msgs::Path &path_msg,geometry_msgs::PoseStamped &pose_msg)
{   
    if(ros::Time::now().toSec()-last_path_time<0.05) return;
    last_path_time=ros::Time::now().toSec();
    path_msg.header.stamp = ros::Time::now(); 
    path_msg.header.frame_id ="base_link";
    path_msg.poses.push_back(pose_msg);
    if(path_msg.poses.size()>=200) path_msg.poses.erase(path_msg.poses.begin());
}

void stop_trajectory()
{
    path.poses.clear();
    trajectory_pub.publish(path);
    follow_square=false;
    follow_elipse=false;

}

void square()
{
     double tt=2;
     double dur = ros::Time::now().toSec()-tbg;
 
 if (dur>tt)
 {
   point_number=(point_number+1)%4;
   tbg=ros::Time::now().toSec();
   double dur = ros::Time::now().toSec()-tbg;
    for (int i=0; i<3; i++)
    {
        prev_pos[i]=pos[i];
        pos[i]=sqr[point_number][i];
    }
 }
 else {

	for(int i=0;i<3;i++) {		
			fig_pos[i]=prev_pos[i]+((pos[i]-prev_pos[i])/tt)*dur;
		}	
		
		geometry_msgs::PoseStamped msg;
		fillmsg(msg,fig_pos);
		oint_pub.publish(msg);
		
		fillpathmsg(path,msg);
		trajectory_pub.publish(path);
		
    }
}

void elipse()
{
     double tt=0.1;
     double dur = ros::Time::now().toSec()-tbg;
 
 if (dur>tt)
 {
   
   tbg=ros::Time::now().toSec();
   
   double dur = ros::Time::now().toSec()-tbg;
    for (int i=0; i<3; i++)
    {
        prev_pos[i]=pos[i];
    }
    gen_elipse_point(pos[0], pos[1], pos[2]);

 }
 else {

	for(int i=0;i<3;i++) {		
			fig_pos[i]=prev_pos[i]+((pos[i]-prev_pos[i])/tt)*dur;
		}	
		
		geometry_msgs::PoseStamped msg;
		fillmsg(msg,fig_pos);
		oint_pub.publish(msg);
		
		fillpathmsg(path,msg);
		trajectory_pub.publish(path);
		
    }
}

bool set_trajectory(lab5::oint_trajectory::Request &req, lab5::oint_trajectory::Response &res) {
    
    if (req.type=="square")
    {
    stop_trajectory();
    follow_square=true;
    follow_elipse=false;
    
     for (int i=0; i<3; i++)
	{
	    pos[i]=sqr[0][i];
	
	}
	point_number=1;
	ros::Rate loop_rate(RATE);
	double begin=ros::Time::now().toSec();
	double duration=ros::Time::now().toSec()-begin;
	czas=2;
	
     while(duration<=czas) {	
	
		for(int i=0;i<3;i++) {		
			fig_pos[i]=prev_pos[i]+((pos[i]-prev_pos[i])/czas)*duration;
		}	
		
		geometry_msgs::PoseStamped msg;
		fillmsg(msg,fig_pos);
		oint_pub.publish(msg);

		loop_rate.sleep();
		duration=ros::Time::now().toSec()-begin;
	}
	
	for(int i=0; i<3; i++)
	{   
	    prev_pos[i]=pos[i];
	    pos[i]=sqr[1][i];
	}
   
	tbg=ros::Time::now().toSec();
	res.status="Following square";
	
	}
	
	else if (req.type=="elipse")
      {
        stop_trajectory();
        follow_square=false;
        follow_elipse=true;
        
        theta=0;
        gen_elipse_point(pos[0], pos[1], pos[2]);
        
	    ros::Rate loop_rate(RATE);
	    double begin=ros::Time::now().toSec();
	    double duration=ros::Time::now().toSec()-begin;
	    czas=2;
	
         while(duration<=czas) {	
	
		    for(int i=0;i<3;i++) {		
			    fig_pos[i]=prev_pos[i]+((pos[i]-prev_pos[i])/czas)*duration;
		    }	
		
		    geometry_msgs::PoseStamped msg;
		    fillmsg(msg,fig_pos);
		    oint_pub.publish(msg);

		    loop_rate.sleep();
		    duration=ros::Time::now().toSec()-begin;
	    }
	
	    for(int i=0; i<3; i++)
	    {   
	        prev_pos[i]=pos[i];
	        
	    }
        gen_elipse_point(pos[0], pos[1], pos[2]);
       
	    tbg=ros::Time::now().toSec();
	    res.status="Following elipse";
	
	  }
	else if (req.type=="stop")
	{
	    stop_trajectory();
	    res.status="Stopped";
	}
	else{
	    res.status="trajectory not recognized";
	    
	}
	return true;
}

bool interpolate(lab5::oint_point::Request &req, lab5::oint_point::Response &res) {
	
	stop_trajectory();
	
	pos[0]=req.x;
	pos[1]=req.y;
	pos[2]=req.z;
    
	czas=req.time;

	bool error=false;
	
	
	if(czas<=0) {
		ROS_ERROR_STREAM("czas poza zakresem");
		error=true;
	}
	
	if(error) {
		res.status="Zadano niepoprawne parametry";  
		return true;
	}
	
	
	
	
	double begin=ros::Time::now().toSec();
	double duration=ros::Time::now().toSec()-begin;

	ros::Rate loop_rate(RATE);

	double new_pos[3];
	double old_pos[3];
	bool er=false;
	
	while(duration<=czas) {	
	
		for(int i=0;i<3;i++) {
		    old_pos[i]=new_pos[i];	
			new_pos[i]=prev_pos[i]+((pos[i]-prev_pos[i])/czas)*duration;
		}	
		
		if(is_out_of_range(new_pos[0],new_pos[1],new_pos[2]))
		{
		 ROS_ERROR_STREAM("Out of range");
		 for(int i=0; i<3; i++)
        {   
    	    prev_pos[i]=old_pos[i];
	    }
	    er=true;
		 break;
		 
		}
		
		
		geometry_msgs::PoseStamped msg;
		fillmsg(msg,new_pos);
		oint_pub.publish(msg);

		loop_rate.sleep();
		duration=ros::Time::now().toSec()-begin;
	}
	if(er!=true)
		{
	
	for(int i=0; i<3; i++)
	{   
	    prev_pos[i]=new_pos[i];
	}
	}
	res.status="Zakonczono interpolacje";
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "oint");
	ros::NodeHandle n;
	ros::ServiceServer point_service = n.advertiseService("oint_point", interpolate);
	ros::ServiceServer trajectory_service = n.advertiseService("oint_trajectory", set_trajectory);
	oint_pub=n.advertise<geometry_msgs::PoseStamped>("oint_pos",1);
	trajectory_pub=n.advertise<nav_msgs::Path>("trajectory",1);
	
	ros::Rate loop_rate(RATE);
	
    double in[]={0.2, 0, 0.4};
    for(int i=0;i<3;i++)
    {
    prev_pos[i]=in[i];
    }
	geometry_msgs::PoseStamped msg;
    fillmsg(msg,in);
    oint_pub.publish(msg);
	
	ROS_INFO("Ready to interpolate.");
	
	stop_trajectory();
	
	 
	while(ros::ok())
	{
	
    if (follow_square) square();
    if (follow_elipse) elipse();
    ros::spinOnce();
    loop_rate.sleep();
	
	}

  return 0;
}
