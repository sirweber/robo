#include <stdio.h>
#include <signal.h>
#include <ros/ros.h>

void quit(int sig)
{
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "telepresence88");
  ros::NodeHandle n;
  signal(SIGINT,quit);

  ros::spin();

  while(true)
  {
    
  }
      
  return(0);
}

