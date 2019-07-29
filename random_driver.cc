#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <stdlib.h> 

int main(int argc, char **argv) {
     //Initializes ROS, and sets up a node
     ros::init(argc, argv, "random_human_commands");
     ros::NodeHandle nh;
     ros::NodeHandle nh_two;
     ros::NodeHandle nh_three;
     ros::NodeHandle nh_four;
     //this is a comment


     //Creates the publisher, and tells it to publish
     //to the husky/cmd_vel topic, with a queue size of 100
     ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("/animated_box0/target_goal", 100);
     ros::Publisher pub_two=nh_two.advertise<geometry_msgs::Twist>("/animated_box1/target_goal", 100);
     ros::Publisher pub_three=nh_three.advertise<geometry_msgs::Twist>("/animated_box2/target_goal", 100);
     ros::Publisher pub_four=nh_four.advertise<geometry_msgs::Twist>("/animated_box3/target_goal", 100);

     //Sets up the random number generator
     srand(time(0));

     //Sets the loop to publish at a rate of 10Hz
     ros::Rate rate(0.1);

       while(ros::ok()) {
          //Declares the message to be sent
          geometry_msgs::Twist msg;
          geometry_msgs::Twist msg_two;
          geometry_msgs::Twist msg_three;
          geometry_msgs::Twist msg_four;
           //Random x value between -2 and 2
           msg.linear.x=15*double(rand())/double(RAND_MAX)-2;
           msg_two.linear.x=15*double(rand())/double(RAND_MAX)-2;
           msg_three.linear.x=15*double(rand())/double(RAND_MAX)-2;
           msg_four.linear.x=15*double(rand())/double(RAND_MAX)-2;
           //Random y value between -3 and 3
           //msg.angular.z=6*double(rand())/double(RAND_MAX)-3;
           msg.linear.y=15*double(rand())/double(RAND_MAX)-2;
           msg_two.linear.y=15*double(rand())/double(RAND_MAX)-3;
           msg_three.linear.y=15*double(rand())/double(RAND_MAX)-3;
           msg_four.linear.y=15*double(rand())/double(RAND_MAX)-3;
           //Publish the message
           pub.publish(msg);
           pub_two.publish(msg_two);
           pub_three.publish(msg_three);
           pub_four.publish(msg_four);

          //Delays untill it is time to send another message
          rate.sleep();
         }
}

