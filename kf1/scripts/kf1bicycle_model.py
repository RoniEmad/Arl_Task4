#!/usr/bin/env python
import time
import math
import rospy
from std_msgs.msg import String
from navigation_task_one.msg import Inputs,Outputs
from geometry_msgs.msg import Pose,PoseArray
from squaternion import Quaternion
print('bicycle model node')
topic2='/arraypose'
pub2=rospy.Publisher(topic2, PoseArray, queue_size=10)


myPosearray=PoseArray()
myPosearray.header.frame_id = "neck"
Maxposearray=200
count=0

X=0
Y=0
Theta=0
#Theta=0
L=10
lr=L/2
time_nanoseclast=0
time_nanosec=0
firsttime=0
def kinematic_model(v,delta):
    global Xdot,Ydot,Thetadot
    Xdot=v*math.cos(Theta)
    Ydot=v*math.sin(Theta)
    Thetadot=v*math.tan(delta)/L
    
    #print(Xdot)
    #print(Ydot)
    #print(Thetadot)
def euler_discretization():
    global X,Xdot,Y,Ydot,Theta,Thetadot,time_nanosec,time_nanoseclast,firsttime
    if(firsttime==0):
        time_nanoseclast = time.time_ns()
        firsttime=1
    time_nanosec = time.time_ns()
    #print(time_nanosec)
    dt=(time_nanosec-time_nanoseclast)*10**-9
    #print(dt)
    X = X + dt*Xdot
    Y = Y + dt*Ydot
    Theta = Theta + dt*Thetadot
    time_nanoseclast = time_nanosec
    if Theta>2*math.pi:
    	Theta=Theta-2*math.pi
    elif Theta<0:
    	Theta=Theta+2*math.pi
def publish_outputs():
    global X,Y,Theta,count
    myPose=Pose()
    myPose.position.x=X
    myPose.position.y=Y
    myPose.position.z=0
    quat=Quaternion.from_euler(0, 0, Theta)
    #print(quat)
    myPose.orientation.x=quat[1]
    myPose.orientation.y=quat[2]
    myPose.orientation.z=quat[3]
    myPose.orientation.w=quat[0]
    #print(myPose.orientation)
    #i=0
    myPosearray.poses.append(myPose)
    #count += 1
    #if count>Maxposearray:
   # 	myPosearray.poses.pop(0)
    #for m in myPosearray.poses:
    #    i+=1
    #print(i)
    pub2.publish(myPosearray)
    
    Pose2 = Outputs()
    Pose2.X=X
    Pose2.Y=Y
    Pose2.Theta=Theta
    pub.publish(Pose2)
    #print(Pose2)

def callback(data):
    v=data.speed
    delta=data.steering_angle
    kinematic_model(v,delta)
    euler_discretization()
    publish_outputs()

def subscriber():
    global pub
    pub = rospy.Publisher('position', Outputs, queue_size=10)
    rospy.init_node('bicycle_model')
    rospy.Subscriber('input', Inputs, callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()
