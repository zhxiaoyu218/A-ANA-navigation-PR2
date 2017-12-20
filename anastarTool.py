#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW2 for EECS 598 Motion Planning
#code based on the simplemanipulation.py example
#### YOUR IMPORTS GO HERE ####
import math
from copy import deepcopy
stepx = 0.0;
stepy = 0.0;
theta_step = 0.0;


class Node:
    G = 1000000;
    E = 1000000;
    def __init__(self, x, y,theta):
      self.x = x;
      self.y = y;
      self.theta = theta;
      self.parent = None;
      self.g = 1000000;
      self.e = 0.;
      self.h = 0.;
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
	result = set();
	for i in range(-1,2):
	 for j in range(-1,2):
	   if(i,j) != (0,0):
		 result.add(Node(node.x+stepx*i,node.y+stepy*j,node.theta) );
	result.add(Node(node.x,node.y,node.theta+theta_step) );
	result.add(Node(node.x,node.y,node.theta-theta_step) );
	
	
	return result;


def GoalCheck(GoalNode,current):
	if math.fabs(GoalNode.x - current.x)<0.01 and math.fabs(GoalNode.y - current.y) <0.01 and math.fabs(GoalNode.theta -current.theta) <0.001 :
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
	a = abs(node1.x-node2.x)/stepx;
	b = abs(node1.y-node2.y)/stepy;
	c = math.fabs((node1.theta-node2.theta)/theta_step);
	return a+b+c;
    

def euclidean(node1,node2):  
	return math.sqrt( math.pow((node2.x-node1.x)/stepx,2)+ math.pow((node2.y-node1.y)/stepy,2) + math.pow( round((node1.theta/theta_step-node2.theta/theta_step),1),2 )   );

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

def improveSolution(openSet,env,robot,handles,GoalNode):
	closeSet = set();
	while len(openSet) != 0:
		current = max(openSet,key=lambda o:o.e);
		openSet.remove(current);
		closeSet.add(deepcopy(current));
		if current.e < Node.E:
			Node.E = current.e;
		if GoalCheck(GoalNode,current):
			print("Goal is found !!!!!!!!!!!!!!");
			Node.G = current.g;
			return current;
		neighbor = FindNeighbor8(current);
		for next in neighbor:
			if nodeIsInSet(next,closeSet):##### g value
				continue;
		  	if check_collision(next,env,robot):
				handles.append(env.plot3(points=[next.x,next.y,0],pointsize=0.05,colors=[1,0,0],drawstyle=1))		
		  	else:
				newCost= current.g + MoveCost(current,next); 	
				if newCost < next.g:
					next.g = newCost;
					#h = manhattan(next,GoalNode);
					h = euclidean(next,GoalNode);
					next.h = h;
					
		 			if next.g + h < Node.G:
						temp = nodeIsInOpenSet(next,openSet);
						if temp == None:							
							next.e =  (Node.G - next.g)/h;
							next.parent = current;
							openSet.add(next);
							handles.append(env.plot3(points=[next.x,next.y,0],pointsize=0.05,colors=[0,0,1],drawstyle=1))
						else:
							openSet.remove(temp);
							next.parent = current;
							next.e =  (Node.G - next.g)/h;
							openSet.add(next);	 
					
	return None;
def OPENupdate(openset):
	newset = set();
	for temp in openset:
		temp.e = (Node.G - temp.g)/temp.h;
		if temp.g+temp.h <Node.G:
			newset.add(temp);
	return newset;

#### END OF YOUR IMPORTS ####
	


