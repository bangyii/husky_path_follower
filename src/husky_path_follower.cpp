/*
 * husky_path_follower.c
 *
 *  Created on: 30 Mar 2020
 *      Author: by
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

//Declare functions
float det(float mat[9]);
void quad_fit(float points[2][2], float grad, float *a, float *b, float *c);
void path_gen(float points[6][2], visualization_msgs::Marker *line);
void bezier(float points[6][2], visualization_msgs::Marker *line);

//Declare global variables
float bot_x, bot_y, bot_yaw;
float target_x, target_y, target_yaw;
visualization_msgs::Marker line, targetPoint, currentPoint;
int pointCounter = 0;


void odomCallback(const nav_msgs::Odometry::ConstPtr &msg){
	geometry_msgs::Point p;
	bot_x = p.x = msg->pose.pose.position.x;
	bot_y = p.y = msg->pose.pose.position.y;
	bot_yaw = 2 * acos(msg->pose.pose.orientation.w) * msg->pose.pose.orientation.z/abs(msg->pose.pose.orientation.z);
	p.z = 0;
	currentPoint.points.push_back(p);
}

void pathUpdateCallback(const ros::TimerEvent& event){
	//Skip update if car is still very far behind
	if(sqrt((target_x - bot_x)*(target_x - bot_x) + (target_y - bot_y)*(target_y - bot_y)) > 1.5)
		return;

	//Update target points
	if(pointCounter < line.points.size()){
	target_x = line.points[pointCounter].x;
	target_y = line.points[pointCounter].y;

	//Calculate target yaw by getting tangent at point
//	//Equivalent to getting angle of tangent at target point
//	float deriv = (line.points[pointCounter+1].y - line.points[pointCounter].y)/
//			(line.points[pointCounter+1].x - line.points[pointCounter].x);
//
//	if((line.points[pointCounter+1].y - line.points[pointCounter].y) < 0 &&
//			(line.points[pointCounter+1].x - line.points[pointCounter].x) < 0){
//		deriv = -deriv;
//			}

	//Ignore last derivative to prevent NaN
	if(pointCounter + 2 < line.points.size())
		target_yaw = atan2((line.points[pointCounter+1].y - line.points[pointCounter].y),
							(line.points[pointCounter+1].x - line.points[pointCounter].x));

	//Update target point markers
	geometry_msgs::Point _point;
	_point.x = target_x;
	_point.y = target_y;
	_point.z = 0;

	targetPoint.points.push_back(_point);
	}

	pointCounter++;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "husky_path_follower");
  ros::NodeHandle nh;
  ros::Publisher line_pub = nh.advertise<visualization_msgs::Marker>("line_markers",10);	//Line publisher for visualization
  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("husky_velocity_controller/cmd_vel",10);
  ros::Subscriber odom_sub = nh.subscribe("odometry/filtered", 100, odomCallback);

  ros::Rate r(60);

  //Create timer to update target position periodically, every 100ms
  float moveSpeed = 0.1;
  ros::Timer timer  = nh.createTimer(ros::Duration(moveSpeed), pathUpdateCallback);

  //Points to interpolate
  std::vector<float> point1, point2, point3, point4, point5, point6;
  if(!nh.getParam("husky_path_follower/point1", point1)) ROS_INFO("Retrieve failed");
  if(!nh.getParam("husky_path_follower/point2", point2)) ROS_INFO("Retrieve failed");
  if(!nh.getParam("husky_path_follower/point3", point3)) ROS_INFO("Retrieve failed");
  if(!nh.getParam("husky_path_follower/point4", point4)) ROS_INFO("Retrieve failed");
  if(!nh.getParam("husky_path_follower/point5", point5)) ROS_INFO("Retrieve failed");
  if(!nh.getParam("husky_path_follower/point6", point6)) ROS_INFO("Retrieve failed");

  float points[6][2] = {
		  {point1[0],point1[1]},
		  {point2[0],point2[1]},
		  {point3[0],point3[1]},
		  {point4[0],point4[1]},
		  {point5[0],point5[1]},
		  {point6[0],point6[1]}
  };

  //Initialize marker object to show points in rvis
  line.header.frame_id = targetPoint.header.frame_id = currentPoint.header.frame_id = "odom";
  line.header.stamp = targetPoint.header.stamp = currentPoint.header.stamp = ros::Time::now();
  line.ns = "line";
  targetPoint.ns = currentPoint.ns = "point";
  line.action = targetPoint.action = currentPoint.action = visualization_msgs::Marker::ADD;
  line.pose.orientation.w = targetPoint.pose.orientation.w = currentPoint.pose.orientation.w = 1.0;
  line.id = 1;
  targetPoint.id = 2;
  currentPoint.id = 3;
  line.type = visualization_msgs::Marker::LINE_STRIP;
  targetPoint.type = currentPoint.type = visualization_msgs::Marker::POINTS;

  line.scale.x =0.02;
  line.color.r = 1;
  line.color.a = 1;

  targetPoint.scale.x = currentPoint.scale.x = 0.1;
  targetPoint.scale.y = currentPoint.scale.y = 0.1;
  targetPoint.color.b = 1;
  currentPoint.color.g = 1;
  targetPoint.color.a = currentPoint.color.a = 1;

  //Generate path
  bool Bezier;
  if(!nh.getParam("husky_path_follower/Bezier", Bezier)) ROS_INFO("Retrieve failed");

  //Bezier curve if true
  if(Bezier) bezier(points, &line);
  else path_gen(points, &line);

  //Create bot velocity object
  geometry_msgs::Twist botVel;
  //Gains
  float linearGain = 2.5, angularGain = 3;
  float yawGain;
  if(!nh.getParam("husky_path_follower/linearGain", linearGain)) ROS_INFO("Retrieve failed");
  if(!nh.getParam("husky_path_follower/angularGain", angularGain)) ROS_INFO("Retrieve failed");

  //Sleep path following node until gazebo/rviz loaded to see progress
  ros::Duration(5).sleep();

  while(ros::ok()){
	  //Publish line and points for visualization
	  line_pub.publish(line);
	  line_pub.publish(targetPoint);
	  line_pub.publish(currentPoint);

	  //Calculate velocity
	  //Slow down car when facing far from tangent path at target point
	  //Also helps prevent overshooting too much during sharp turn
	  if(abs(target_yaw - bot_yaw) < abs(target_yaw + bot_yaw)){
		  yawGain = 0.08 * pow(2.71828, -5*(abs(target_yaw - bot_yaw)/3.14159 - 0.5)) + -0.00;
	  }

	  else{
		  yawGain = 0.08 * pow(2.71828, -5*(abs(target_yaw + bot_yaw)/3.14159 - 0.5)) + -0.00;
	  }

	  //Speed scales with distance between points
	  botVel.linear.x = linearGain * yawGain *
			  	  	  sqrt((target_x - bot_x)*(target_x - bot_x) + (target_y - bot_y)*(target_y - bot_y));


	  //Angular speed scales with difference in angle to point at target point
	  //Target theta is the heading that the bot should face, eg angle between target
	  //point and current bot location
	  float target_theta = atan2((target_y - bot_y), (target_x - bot_x));
	  if(abs(target_theta - bot_yaw) < abs(target_theta + bot_yaw)){
		  botVel.angular.z = angularGain * (target_theta - bot_yaw);
	  }

	  else{
		  botVel.angular.z = angularGain * (target_theta + bot_yaw);
	  }

	  //Publish velocity to husky
	  cmd_vel_pub.publish(botVel);
	  ROS_INFO("%f, %f", botVel.linear.x, botVel.angular.z);
//	  ROS_INFO("%f, %f", target_theta, bot_yaw);

	  ros::spinOnce();
	  r.sleep();
  }

  return 0;
}

float det(float mat[9]){
	//Calculates determinant of 3x3 matrix. Index is show below
	// 0 1 2
	// 3 4 5
	// 6 7 8

	float det_1 = mat[0]*(mat[4]*mat[8] - mat[7]*mat[5]);
	float det_2 = mat[1]*(mat[3]*mat[8] - mat[6]*mat[5]);
	float det_3 = mat[2]*(mat[3]*mat[7] - mat[6]*mat[4]);
	return det_1 - det_2 + det_3;
}

void quad_fit(float points[2][2], float grad, float *a, float *b, float *c){
//Finds quadratic equation given starting gradient constraint
  float x_1 = points[0][0], x_2 = points[1][0];
  float y_1 = points[0][1], y_2 = points[1][1];

  //Cramer's Rule for solving simultaneous
  float data_D[] = {x_1*x_1, x_1, 1,
		  	  x_2*x_2, x_2, 1,
			  2*x_1, 1, 0};

  float det_D = det(data_D);

  float data_Da[] = {y_1, x_1, 1,
		  	  	  y_2, x_2, 1,
				  grad, 1, 0};

  float det_Da = det(data_Da);

  float data_Db[] = {x_1*x_1, y_1, 1,
		  	  	  x_2*x_2, y_2, 1,
				  2*x_1, grad, 0};

  float det_Db = det(data_Db);

  float data_Dc[] = {x_1*x_1, x_1, y_1,
		  	  x_2*x_2, x_2, y_2,
			  2*x_1, 1, grad};
  float det_Dc = det(data_Dc);

  //Assign quadratic coefficients
  *a = det_Da/det_D;
  *b = det_Db/det_D;
  *c = det_Dc/det_D;
}

void path_gen(float points[6][2], visualization_msgs::Marker *line){
//Finds continuous piecewise quadratic path that passes through 5 points
//Points must have an increasing x value

	float last_derivative = 0, x_interval;
	float a, b, c;
	float no_points;
	float points_array[2][2];

	for(int i = 0; i < 5; i ++){
		no_points = 50;

		//Find interval between each interpolated point
		x_interval = (points[i+1][0] - points[i][0]) / no_points;

		//Assign points array
		points_array[0][0] = points[i][0];
		points_array[0][1] = points[i][1];
		points_array[1][0] = points[i+1][0];
		points_array[1][1] = points[i+1][1];

		//Calculate a b c coefficients
		quad_fit(points_array, last_derivative, &a, &b, &c);

		//Assign points to line
		for(int j = 0; j < no_points; j++){
			geometry_msgs::Point p;
			p.x = points[i][0] + j*x_interval;
			p.y = a*p.x*p.x + b*p.x + c;
			p.z = 0;

			line->points.push_back(p);
		}

		//Assign derivative of endpoint
		last_derivative = 2*a*points[i+1][0] + b;
	}
}

void bezier(float points[6][2], visualization_msgs::Marker *line){
	//Function to create bezier curve joining 6 consecutive points
	float pointArray[2][2] = {
			{points[0][0],points[0][1]},
			{points[1][0],points[1][1]}
	};

	//Initalize first control point to be tangent to x-axis
	float ctrPoint[2] = {points[1][0]*(float)0.9, 0};
	float gradCtr = 0, ctrDist = 5, c = 0, gradPoints = 0;
	bool more = false;
	geometry_msgs::Point p;


	//Bezier curves
	for(int k = 0; k < 5; k++){
		if(k > 0){
			gradCtr = (pointArray[1][1] - ctrPoint[1])/(pointArray[1][0] - ctrPoint[0]);

			//Find line and gradient between control point and point 2
			gradPoints = (pointArray[1][1] - pointArray[0][1])/(pointArray[1][0] - pointArray[0][0]);
			c = pointArray[1][1] - gradPoints * pointArray[1][0];

			//Check which side old control point lies
			if(ctrPoint[1] > gradPoints * ctrPoint[0] + c) more = true;
			else more = false;

			//Ensure control point is on opposite side of line between points
			//with respect to old control point
			ctrPoint[0] = pointArray[1][0] + ctrDist/sqrt(gradCtr*gradCtr + 1);
			ctrPoint[1] = gradCtr*(ctrPoint[0] - pointArray[1][0]) + pointArray[1][1];

			//Check if old and new control point are both less than line
			if(ctrPoint[1] < gradPoints * ctrPoint[0] + c && !more){
				ctrPoint[0] = pointArray[1][0] - ctrDist/sqrt(gradCtr*gradCtr +1);
				ctrPoint[1] = gradCtr*(ctrPoint[0] - pointArray[1][0]) + pointArray[1][1];
			}


			//Assign points for use
			pointArray[0][0] = points[k][0];
			pointArray[0][1] = points[k][1];
			pointArray[1][0] = points[k+1][0];
			pointArray[1][1] = points[k+1][1];
		}

		//Get Bezier curve
		for(float i = 0; i < 1; i+= 0.02){
			p.x = (1-i)*(1-i)* pointArray[0][0] + 2*(1-i)*(i)*ctrPoint[0] + i*i*pointArray[1][0];
			p.y = (1-i)*(1-i)* pointArray[0][1] + 2*(1-i)*(i)*ctrPoint[1] + i*i*pointArray[1][1];
			p.z = 0;
			line->points.push_back(p);
		}
	}
}
