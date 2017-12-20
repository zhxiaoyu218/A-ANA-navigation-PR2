#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW2 for EECS 598 Motion Planning
#code based on the simplemanipulation.py example
#### YOUR IMPORTS GO HERE ####
import math
stepx = 0.0;
stepy = 0.0;
theta_step = 0.0;
class Node:
    def __init__(self, x, y,theta):
      self.x = x;
      self.y = y;
      self.theta = theta;
      self.parent = None;
      self.G = 0.;
      self.H = 0.;
    def __repr__(self):
       return repr((self.x, self.y,self.theta))



def FindNeighbor4(node):

	final = set();
	final.add(Node(node.x+stepx,node.y,node.theta));
	final.add(Node(node.x-stepx,node.y,node.theta));
	final.add(Node(node.x,node.y+stepy,node.theta));
	final.add(Node(node.x,node.y-stepy,node.theta));
	final.add(Node(node.x,node.y,node.theta+theta_step));
	final.add(Node(node.x,node.y,node.theta-theta_step));
	return final;

def FindNeighbor8(node):
	'''
	result = set();
	for i in range(-1,2):
	 for j in range(-1,2):
	  for k in range(-1,2):
	   if(i,j,k) != (0,0,0):
		 result.add(Node(node.x+stepx*i,node.y+stepy*j,node.theta+theta_step*k) );
	'''
	result = set();
	for i in range(-1,2):
	 for j in range(-1,2):
	   if(i,j) != (0,0):
		 result.add(Node(node.x+stepx*i,node.y+stepy*j,node.theta) );
	result.add(Node(node.x,node.y,node.theta+theta_step) );
	result.add(Node(node.x,node.y,node.theta-theta_step) );
	
	return result;

def GoalCheck(GoalNode,current):
	if math.fabs(GoalNode.x - current.x)<0.001 and math.fabs(GoalNode.y - current.y) <0.001 and math.fabs(GoalNode.theta -current.theta) <0.001 :
		return True;
	return False;

def nodeIsInSet(node,MySet):
	for temp in MySet:	
		if (node.x == temp.x) and (node.y == temp.y) and (round(node.theta,3) == round(temp.theta,3)):
			return True;
	return False;

def nodeIsInOpenSet(node,MySet):
	for temp in MySet:	
		if (node.x == temp.x) and (node.y == temp.y) and (round(node.theta,3) == round(temp.theta,3)):
			return temp;
	return None;  
  
def manhattan(node1,node2):  
	a = math.fabs(node1.x-node2.x)/stepx;
	b = math.fabs(node1.y-node2.y)/stepy;
	c = round(math.fabs(node1.theta/theta_step-node2.theta/theta_step));
	return a+b+c;
    

def euclidean(node1,node2):  
	return math.sqrt( math.pow((node2.x-node1.x)/stepx,2)+ math.pow((node2.y-node1.y)/stepy,2) +math.pow(round(node1.theta/theta_step-node2.theta/theta_step),2));

def check_collision(node,env,robot):  
    robot.SetActiveDOFValues([node.x,node.y,node.theta])
    return env.CheckCollision(robot)

def computePathAry(node):
	path = [];
	temp = node;
	while temp != None:
		path.append(temp);
		temp =temp.parent;
	return path;

def MoveCost(node1,node2):	
	return euclidean(node1,node2);
#### END OF YOUR IMPORTS ####
	


