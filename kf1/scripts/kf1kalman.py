#!/usr/bin/env python
import time
import math
import rospy
import numpy as np
from std_msgs.msg import String
from navigation_task_one.msg import Inputs,Outputs
from geometry_msgs.msg import Pose,PoseArray
from squaternion import Quaternion
print('Kalman node')
topic2='/kalmanmodelarraypose'
pub2=rospy.Publisher(topic2, PoseArray, queue_size=10)


myPosearray=PoseArray()
myPosearray.header.frame_id = "neck"
Maxposearray=200
count=0
arrayX=[]
arrayY=[]
arrayTheta=[]
X=0
Y=0
Theta=0
#Theta=0
L=10
lr=L/2
time_nanoseclast=0
time_nanosec=0
firsttime=0
state_estimate_k_minus_1 = np.array([0.0,0.0,0.0])
obs_vector_z_k = np.array([0.0,0.0,0.0])
def kinematic_model(v,delta):
    global X,Xdot,Y,Ydot,Theta,Thetadot,firsttime,time_nanosec,time_nanoseclast,obs_vector_z_k,state_estimate_k_minus_1
    Xdot=v*math.cos(Theta)
    Ydot=v*math.sin(Theta)
    Thetadot=v*math.tan(delta)/L
    
    if(firsttime==0):
        time_nanoseclast = time.time_ns()
        firsttime=1

    time_nanosec = time.time_ns()
    
    dk=(time_nanosec-time_nanoseclast)*10**-9
    
 
    
    X = state_estimate_k_minus_1[0]
    Y = state_estimate_k_minus_1[1]
    Theta = state_estimate_k_minus_1[2]
    
    if obs_vector_z_k[2]-state_estimate_k_minus_1[2]>(7/4*math.pi):
        state_estimate_k_minus_1[2]+=2*math.pi
    elif obs_vector_z_k[2]-state_estimate_k_minus_1[2]<(-7/4*math.pi):
        state_estimate_k_minus_1[2]-=2*math.pi
    
    #if Theta>2*math.pi:
   # 	Theta=Theta-2*math.pi
    #elif Theta<0:
    #	Theta=Theta+2*math.pi
    arrayX.append(X)
    arrayY.append(Y)
    arrayTheta.append(Theta)
    print(arrayTheta)
    control_vector_k_minus_1 = np.array([Xdot,Ydot,Thetadot])
    P_k_minus_1 = np.array([[0.1,  0,   0],
                            [  0,0.1,   0],
                            [  0,  0, 0.1]])
    optimal_state_estimate_k, covariance_estimate_k = ekf(
            obs_vector_z_k, # Most recent sensor measurement
            state_estimate_k_minus_1, # Our most recent estimate of the state
            control_vector_k_minus_1, # Our most recent control input
            P_k_minus_1, # Our most recent state covariance matrix
            dk) # Time interval
    
    time_nanoseclast = time_nanosec
        # Get ready for the next timestep by updating the variable values
    state_estimate_k_minus_1 = optimal_state_estimate_k
    #print(state_estimate_k_minus_1)
    P_k_minus_1 = covariance_estimate_k
    


A_k_minus_1= np.array([[1.0,   0,   0],
                       [  0, 1.0,   0],
                       [  0,   0, 1.0]])
process_noise_v_k_minus_1 = np.array([0.01,0.01,0.003])
Q_k = np.array([[1.0,   0,   0],
                [  0, 1.0,   0],
                [  0,   0, 1.0]])
H_k = np.array([[1.0,  0,   0],
                [  0,1.0,   0],
                [  0,  0, 1.0]])
R_k = np.array([[1.0,   0,    0],
                [  0, 1.0,    0],
                [  0,    0, 1.0]])
sensor_noise_w_k = np.array([0.07,0.07,0.04])
def getB(yaw, deltak):
    B = np.array([[deltak,      0,      0],
                  [     0, deltak,      0],
                  [     0,      0, deltak]])
    return B

def ekf(z_k_observation_vector, state_estimate_k_minus_1, 
        control_vector_k_minus_1, P_k_minus_1, dk):
    """
    Extended Kalman Filter. Fuses noisy sensor measurement to 
    create an optimal estimate of the state of the robotic system.
         
    INPUT
        :param z_k_observation_vector The observation from the Odometry
            3x1 NumPy Array [x,y,yaw] in the global reference frame
            in [meters,meters,radians].
        :param state_estimate_k_minus_1 The state estimate at time k-1
            3x1 NumPy Array [x,y,yaw] in the global reference frame
            in [meters,meters,radians].
        :param control_vector_k_minus_1 The control vector applied at time k-1
            3x1 NumPy Array [v,v,yaw rate] in the global reference frame
            in [meters per second,meters per second,radians per second].
        :param P_k_minus_1 The state covariance matrix estimate at time k-1
            3x3 NumPy Array
        :param dk Time interval in seconds
             
    OUTPUT
        :return state_estimate_k near-optimal state estimate at time k  
            3x1 NumPy Array ---> [meters,meters,radians]
        :return P_k state covariance_estimate for time k
            3x3 NumPy Array                 
    """
    ######################### Predict #############################
    # Predict the state estimate at time k based on the state 
    # estimate at time k-1 and the control input applied at time k-1.
    state_estimate_k = A_k_minus_1 @ (
            state_estimate_k_minus_1) + (
            getB(state_estimate_k_minus_1[2],dk)) @ (
            control_vector_k_minus_1) + (
            process_noise_v_k_minus_1)
             
    print(f'State Estimate Before EKF={state_estimate_k}')
             
    # Predict the state covariance estimate based on the previous
    # covariance and some noise
    P_k = A_k_minus_1 @ P_k_minus_1 @ A_k_minus_1.T + (
            Q_k)
         
    ################### Update (Correct) ##########################
    # Calculate the difference between the actual sensor measurements
    # at time k minus what the measurement model predicted 
    # the sensor measurements would be for the current timestep k.
    measurement_residual_y_k = z_k_observation_vector - (
            (H_k @ state_estimate_k) + (
            sensor_noise_w_k))
 
    print(f'Observation={z_k_observation_vector}')
             
    # Calculate the measurement residual covariance
    S_k = H_k @ P_k @ H_k.T + R_k
         
    # Calculate the near-optimal Kalman gain
    # We use pseudoinverse since some of the matrices might be
    # non-square or singular.
    K_k = P_k @ H_k.T @ np.linalg.pinv(S_k)
         
    # Calculate an updated state estimate for time k
    state_estimate_k = state_estimate_k + (K_k @ measurement_residual_y_k)
     
    # Update the state covariance estimate for time k
    P_k = P_k - (K_k @ H_k @ P_k)
     
    # Print the best (near-optimal) estimate of the current state of the robot
    print(f'State Estimate After EKF={state_estimate_k}')
 
    # Return the updated state and covariance estimates
    return state_estimate_k, P_k


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
    publish_outputs()
def getobservation(data):
    global obs_vector_z_k
    obs_vector_z_k=np.array([data.X,data.Y,data.Theta])
def subscriber():
    global pub
    pub = rospy.Publisher('kalman_model_position', Outputs, queue_size=10)
    rospy.init_node('kalman_model')
    rospy.Subscriber('observation', Outputs, getobservation)
    rospy.Subscriber('input', Inputs, callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()
