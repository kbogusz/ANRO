#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <stdio.h>
#include <unistd.h>   //_getch
#include <termios.h>

//pobieranie znaku
char getch(){
    
    char buf=0;
    struct termios old={0};
    fflush(stdout);
    if(tcgetattr(0, &old)<0)
        perror("tcsetattr()");
    old.c_lflag&=~ICANON;
    old.c_lflag&=~ECHO;
    old.c_cc[VMIN]=1;
    old.c_cc[VTIME]=0;
    if(tcsetattr(0, TCSANOW, &old)<0)
        perror("tcsetattr ICANON");
    if(read(0,&buf,1)<0)
        perror("read()");
    old.c_lflag|=ICANON;
    old.c_lflag|=ECHO;
    if(tcsetattr(0, TCSADRAIN, &old)<0)
        perror ("tcsetattr ~ICANON");
    return buf;
 }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lab1");


  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

  ros::Rate loop_rate(1000);

  while (ros::ok())
  {
    geometry_msgs::Twist twist;

	char c = getch();

	if(c=='w'){
		twist.linear.x=2.0;        
	}
	if(c=='s'){
		twist.linear.x=-2.0;     
	}
	if(c=='a'){
		twist.angular.z=2.0;
        }
	if(c=='d'){
		twist.angular.z=-2.0;
	}    
    
    chatter_pub.publish(twist);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

