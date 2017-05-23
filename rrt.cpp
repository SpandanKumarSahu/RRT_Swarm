/*
  Before using :
  1. Change the BOT_ID
  2. Change the rows and cols of the image matrix
*/

#include "stdio.h"
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

using namespace cv;
using namespace std;

static const int ROWS = 600;
static const int COLS = 800;
static const int BOT_ID = 0;
static const bool isCompleted = false;

typedef struct
{
  int x;
  int y;
}coordi;

struct Node {
    vector<Node *> children;
    Node *parent;
    coordi position;
};

Node start_node;
Node end_node;
Mat img;
Node* nodes[20000];
int totnodes = 0;
int reached = 0;
int step_size = 10;
int iter = 0;
int src_x, src_y, dest_x, dest_y;
geometry_msgs::PoseArray pub;

void init()
{
    start_node.position.x = src_x;
    start_node.position.y = src_y;
    start_node.parent = NULL;
    for(int i=max(start_node.position.x - 5, 0); i < min(start_node.position.x + 5,ROWS); i++)
    {
      for(int j=max(start_node.position.y - 5, 0); j < min(start_node.position.y + 5, COLS); j++)
      {
        img.at<Vec3b>(i, j)[0] = 255;
        img.at<Vec3b>(i, j)[1] = 0;
        img.at<Vec3b>(i, j)[2] = 0;
      }
    }
    nodes[totnodes++] = &start_node;
    end_node.position.x = dest_x;
    end_node.position.y = dest_y;
    for(int i=max(end_node.position.x - 5, 0); i < mian(end_node.position.x + 5, ROWS); i++)
    {
      for(int j=max(end_node.position.y - 5, 0); j < min(end_node.position.y + 5, COLS); j++)
      {
        img.at<Vec3b>(i, j)[0] = 0;
        img.at<Vec3b>(i, j)[1] = 0;
        img.at<Vec3b>(i, j)[2] = 255;
      }
    }
    srand(time(NULL));
}

float node_dist(coordi p, coordi q)
{
  coordi v;
  v.x = p.x - q.x;
  v.y = p.y - q.y;
  return sqrt(pow(v.x, 2) + pow(v.y, 2));
}

int near_node(Node rnode)
{
  float min_dist = 999.0, dist= node_dist(start_node.position, rnode.position);
  int lnode = 0, i = 0;
  for(i=0; i<totnodes; i++)
  {
    dist = node_dist(nodes[i]->position, rnode.position);
    if(dist<min_dist)
    {
      min_dist = dist;
      lnode = i;
    }
  }
  return lnode;
}

coordi stepping(coordi nnode, coordi rnode)
{
  coordi interm, step;
  float magn = 0.0, x = 0.0, y = 0.0;
  interm.x = rnode.x - nnode.x;
  interm.y = rnode.y - nnode.y;
  magn = sqrt((interm.x)*(interm.x) + (interm.y)*(interm.y));
  x = (float)(interm.x / magn);
  y = (float)(interm.y / magn);
  step.x = (int)(nnode.x + step_size*x);
  step.y = (int)(nnode.y + step_size*y);
  return step;
}

int check_validity_1(coordi p, coordi q)
{
  coordi large, small;
  int i = 0, j1 = 0, j2 = 0;
  double slope;
  if(q.x<p.x)
  {
    small = q;
    large = p;
  }
  else
  {
    small = p;
    large = q;
  }
  if(large.x == small.x)
    return 0;
  slope = ((double)large.y - small.y)/((double)large.x - small.x);
  for(i=small.x+1; i<large.x; i++)
  {
    j1 = (int)((i*slope) - (small.x)*(slope) + small.y);
    j2 = j1 + 1;
    if((i<0) || (i>ROWS) || (j1<0) || (j1>COLS) || (j2<0) || (j2>COLS))
      continue;
    if(((int)img.at<Vec3b>(i, j1)[0] <250) && ((int)img.at<Vec3b>(i, j1)[1] < 250) && ((int)img.at<Vec3b>(i, j1)[2] < 250))
     return 0;
     if(((int)img.at<Vec3b>(i, j2)[0] <250) && ((int)img.at<Vec3b>(i, j2)[1] < 250) && ((int)img.at<Vec3b>(i, j2)[2] < 250))
      return 0;
  }
  return 1;
}

int check_validity_2(coordi p, coordi q)
{
  coordi large, small;
  int i = 0, j1 = 0, j2 = 0;
  double slope;
  if(q.y<p.y)
  {
    small = q;
    large = p;
  }
  else
  {
    small = p;
    large = q;
  }
  if(large.x == small.x)
    return 0;
  slope = ((double)large.y - small.y)/((double)large.x - small.x);
  for(i=small.y+1; i<large.y; i++)
  {
    j1 = (int)(((i - small.y)/slope) + small.x);
    j2 = j1 + 1;
    if((i<0) || (i>ROWS) || (j1<0) || (j1>COLS) || (j2<0) || (j2>COLS))
      continue;
    if(((int)img.at<Vec3b>(j1, i)[0] <250) && ((int)img.at<Vec3b>(j1, i)[1] < 250) && ((int)img.at<Vec3b>(j1, i)[2] < 250))
     return 0;
     if(((int)img.at<Vec3b>(j2, i)[0] <250) && ((int)img.at<Vec3b>(j2, i)[1] < 250) && ((int)img.at<Vec3b>(j2, i)[2] < 250))
      return 0;
  }
  return 1;
}

void draw_path()
{
  Node up, down;
  int breaking = 0;
  down = end_node;
  up = *(end_node.parent);
  while(1)
  {
    //line(img, Point(up.position.y, up.position.x), Point(down.position.y, down.position.x), Scalar(0, 255, 0), 2, 8);
    geometry_msgs::Pose temp;
    temp.position.x = down.position.x;
    temp.position.y = down.position.y;
    pub.poses.push_back(temp);
    if(up.parent == NULL)
      break;
    up = *(up.parent);
    down = *(down.parent);
  }
}

void rrt()
{
  int flag1 = 0, index = 0, flag2 = 0;
  Node* rnode = new Node;
  Node* stepnode = new Node;
  (rnode->position).x = rand() % ROWS + 1;
  (rnode->position).y = rand() % COLS + 1;
  index = near_node(*rnode);
  if((node_dist(rnode->position, nodes[index]->position)) < step_size)
    return;
  else
    stepnode->position = stepping(nodes[index]->position, rnode->position);
  flag1 = check_validity_1(nodes[index]->position, stepnode->position);
  flag2 = check_validity_2(nodes[index]->position, stepnode->position);
  if((flag1 == 1) && (flag2 == 1))
    {
      nodes[totnodes++] = stepnode;
      stepnode->parent = nodes[index];
      (nodes[index]->children).push_back(stepnode);
      line(img, Point((stepnode->position).y, (stepnode->position).x), Point(nodes[index]->position.y, nodes[index]->position.x), Scalar(0, 255, 255), 2, 8);
      for(int i=stepnode->position.x - 2; i < stepnode->position.x + 2; i++)
	{
	  for(int j=stepnode->position.y - 2; j < stepnode->position.y + 2; j++)
	    {
	      if((i<0) || (i>ROWS) || (j<0) || (j>COLS))
		continue;

	      img.at<Vec3b>(i, j)[0] = 0;
	      img.at<Vec3b>(i, j)[1] = 255;
	      img.at<Vec3b>(i, j)[2] = 0;
	    }
	}
      if((check_validity_1(stepnode->position, end_node.position)) && (check_validity_2(stepnode->position, end_node.position)) && (node_dist(stepnode->position,end_node.position) < step_size))
	{
	  reached = 1;
	  nodes[totnodes++] = &end_node;
	  end_node.parent = stepnode;
	  (nodes[totnodes-1]->children).push_back(&end_node);
	  draw_path();
	}
    }
  iter++;
}

void createImage(const AprilTags::AprilTagDetectionArray& msg){
  img = Mat(ROWS, COLS, CV_8UC3, Scalar(0, 0, 0));
  int i = 0;
  int sum_x = 0;
  int sum_y = 0;
  for(std::vector<AprilTags::AprilTagDetection>::const_iterator it = msg->detections.begin(); it != msg->detections.end(); ++it){
    AprilTags::AprilTagDetection temp = *it;
    if(temp.id == BOT_ID){
      src_x = (int) temp.pose.pose.position.x;
      src_y = (int) temp.pose.pose.position.y;
    }
    sum_x += temp.pose.pose.position.x;
    sum_y += temp.pose.pose.position.y;
    ++i;
  }
  dest_x = sum_x/i;
  dest_y = sum_y/i;
}

void callbackRRT(const AprilTags::AprilTagDetectionArray& msg){
  if(!isCompleted){
    createImage(msg); //Should Initialise the img and set src_ and dest_
    init();
    while((reached == 0)){
      rrt();
    }
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseArray>("wayPoints", 1000);
    ros::Rate loop_rate(10);
    chatter_pub.publish(pub);
    isCompleted = true;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("tag_detections", 10, callbackRRT);
  ros::spin();
  waitKey();
  return 0;
}
