#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped,Pose,PoseArray,Pose2D
from visualization_msgs.msg import Marker
from navigation_task_one.msg import Inputs,Outputs
from squaternion import Quaternion
import matplotlib.pyplot as plt
print('pure pursuit controller node')
pathX=[]
pathY=[]
pathTheta=[]
frequency=20
L=10
lr=L/2
subonce=None
v=1
k=1.5
maxld=k*v

rospy.init_node('controller')
topic = 'visualization_marker'
mark = rospy.Publisher(topic, Marker, queue_size=10)

def controller(X,Y,Theta):
    global rate,v
    index=0
    miniiindex=0
    mini=10000
    ld=0
    for i in range(len(pathX)):
        distance=math.sqrt(pow(pathX[i]-X,2)+pow(pathY[i]-Y,2))
        if distance<mini:
            mini=distance
            miniindex=i
        if distance<maxld:
            index=i
            ld=distance
    if ld==0:
        ld=mini
        index=miniindex
    print(ld)
    print(index)
    
    dy=pathY[index]-Y
    dx=pathX[index]-X
    slope=0
    if dx==0 and dy>0:
        vectorangle=math.pi/2
    elif dx==0 and dy<0:
        vectorangle=math.pi*3/2
    else:
        slope=dy/dx
        vectorangle=math.atan(slope)
    
    if dy>0 and dx<0:
        vectorangle+=math.pi
    elif dy<0 and dx>0:
        vectorangle=2*math.pi+vectorangle
    elif dy<0 and dx<0:
        vectorangle+=math.pi
    
    alpha=vectorangle-Theta
    print('alpha= ',alpha)
    if alpha<0:
        alpha+=2*math.pi
    if alpha>(math.pi/2) and alpha<math.pi:
        alpha=math.pi/2
    elif alpha>math.pi and alpha<(3/2*math.pi):
        alpha=3*math.pi/2
    
    sinalpha=math.sin(alpha)

    print(vectorangle,Theta,pathTheta[index],X,pathX[index],Y,pathY[index],slope,alpha,sinalpha)
    delta=math.atan(2*L*sinalpha/ld)
    
    print(sinalpha,ld,alpha,delta)
    
    #v=(len(pathX)-index-1)*0.01
    #if v>1:
    #    v=1
    #if len(pathX)==index+1 and v>0:
    #    v-=0.01
    #    delta=0
    if ld<0.1:
        v=0
    outputdata=Inputs()
    outputdata.speed=v
    outputdata.steering_angle=delta
    pub.publish(outputdata)
    
    
    marker = Marker()
    marker.header.frame_id = "neck"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 0
    marker.pose.position.x = pathX[index]
    marker.pose.position.y = pathY[index]
    marker.pose.position.z = 0
    mark.publish(marker)
    rate.sleep()

def callback(data):
    X=data.X
    Y=data.Y
    Theta=data.Theta
    controller(X,Y,Theta)
def pathcallback(data):
    q=Quaternion()
    count=0
    for po in data.poses:
        pathX.append(po.position.x)
        pathY.append(po.position.y)
        q=Quaternion(po.orientation.w,po.orientation.x,po.orientation.y,po.orientation.z)
        euler=q.to_euler()
        pathTheta.append(euler[2])
    subonce.unregister()

def subscriber():
    global pub,subonce,rate
    pub = rospy.Publisher('input', Inputs, queue_size=10)
    rate=rospy.Rate(frequency)
    subonce = rospy.Subscriber('path',PoseArray,pathcallback)
    rospy.wait_for_message('path',PoseArray)
    rospy.Subscriber('position', Outputs, callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()
