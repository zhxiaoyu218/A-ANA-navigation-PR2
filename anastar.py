#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW2 for EECS 598 Motion Planning
#code based on the simplemanipulation.py example
import time
import openravepy

#### YOUR IMPORTS GO HERE ####
import anastarTool as ana

#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)


	
if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)


    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('data/pr2test2.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);

    with env:
        # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])

        goalconfig = [2.6,-1.3,-pi/2]
        #### YOUR CODE HERE ####
        GoalNode = ana.Node(goalconfig[0],goalconfig[1],goalconfig[2]);
        CloseSet = set();
        OpenSet = set();
        clideSet=set();
        ana.stepx = 0.2;
        ana.stepy = 0.1;
        ana.theta_step = pi/2;
        handles = [];

        ana.Node.G = 10000.;
        ana.Node.E = 10000.;
        str = ana.Node(-3.4,-1.4,0);
        str.g=0;
        #str.h = ana.manhattan(str,GoalNode);
        str.h = ana.euclidean(str,GoalNode);
        str.e = (ana.Node.G - str.g)/str.h;
        OpenSet.add(str);
        print(str.g," " , str.h," ",str.e);
	colorlist = [];
	colorlist.append([0,0,0]);
	colorlist.append([1,0,1]);
	colorlist.append([0,1,0]);
	colorlist.append([0,1,1]);
	colorlist.append([1,0,1]);
	counter = 0;
        
        while len(OpenSet) !=0:
        	Timer_str = time.clock();
        	suboptimal=ana.improveSolution(OpenSet,env,robot,handles,GoalNode);
        	Timer_end = time.clock();
        	if suboptimal == None:
        		print("nothing could be improved, loop will break");
	        	break;
		PathAry = ana.computePathAry(suboptimal);
		for i in PathAry:
        		handles.append(env.plot3(points=[i.x,i.y,0.05],pointsize=0.05,colors=colorlist[counter],drawstyle=1));

        	print("latest cost is ",suboptimal.g," time is ",(Timer_end-Timer_str),"len is",len(OpenSet));
        	OpenSet = ana.OPENupdate(OpenSet);
		counter=counter+1;
        
        PathAry.reverse();
        for i in PathAry:
        	#print(i);
        	handles.append(env.plot3(points=[i.x,i.y,0],pointsize=0.05,colors=[0,0,0],drawstyle=1));

        #### Implement your algorithm to compute a path for the robot's base starting from the current configuration of the robot and ending at goalconfig. The robot's base DOF have already been set as active. It may be easier to implement this as a function in a separate file and call it here.

        #### Draw the X and Y components of the configurations explored by your algorithm


        #### Now that you have computed a path, execute it on the robot using the controller. You will need to convert it into an openrave trajectory. You can set any reasonable timing for the configurations in the path. Then, execute the trajectory using robot.GetController().SetPath(mypath);
        
        traj = RaveCreateTrajectory(env,'');
        config = robot.GetActiveConfigurationSpecification('linear');

        config.AddDeltaTimeGroup();
        traj.Init(config);
        myPath = [ [point.x, point.y,point.theta,i*0.01] for i,point in enumerate(PathAry) ];
	
        for i ,wayPoint in enumerate(myPath):
        	traj.Insert(i,wayPoint,config,True);
        robot.GetController().SetPath(traj);
        #### END OF YOUR CODE ###
    waitrobot(robot)

    raw_input("Press enter to exit...")

