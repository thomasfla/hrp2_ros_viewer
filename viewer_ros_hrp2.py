#!/usr/bin/env python

#Note: 
#  Install https://github.com/stack-of-tasks/dynamic_graph_bridge_msgs for ROS MSGs types
#  set ros master-ros to hrp2c: 
#    export ROS_MASTER_URI=http://hrp2014c:11311
from std_msgs.msg import String
from dynamic_graph_bridge_msgs.msg import Vector
from hrp2014_wrapper import Hrp2014Wrapper
from viewer_utils import Viewer
from pinocchio.rpy import matrixToRpy
import pinocchio as se3
from pinocchio import Quaternion
import os
import numpy as np
import rospy
import subprocess

from  config import *
#~ MESH_PATH = '/opt/openrobots/share'
#~ HRP2_URDF_MODEL = "/opt/openrobots/share/hrp2_14_description/urdf/hrp2_14_reduced.urdf"
#~ READ_FREEFLYER_FROM_AA_SIGNAL = True

#Global variable filled by asyncronous ROS topics
q_glob = np.zeros(37);
ff_xzyquat = np.array([ 0.,0.,0.  ,  0.,0.,0.,1. ])

def callback_robotState(data):
    #read vector q from ros
    q = np.array(data.data)  
    #construct qUrdf from sot convention and eventualy external freeflyer 
    qUrdf = np.zeros(37)
    if READ_FREEFLYER_FROM_AA_SIGNAL:
        global ff_xzyquat
        qUrdf[:7] = ff_xzyquat
    else :
        qUrdf[:3] = q[:3];      
        quatMat = se3.utils.rpyToMatrix(np.matrix(q[3:6]).T)
        quatVec = Quaternion(quatMat);
        qUrdf[3:7]   = np.array(quatVec.coeffs()).squeeze();
    qUrdf[7:11]  = q[18:22]; # chest-head
    qUrdf[11:18] = q[29:]; # larm
    qUrdf[18:25] = q[22:29]; # rarm
    qUrdf[25:31] = q[12:18]; # lleg
    qUrdf[31:]   = q[6:12]; # rleg
    global q_glob
    q_glob=qUrdf #update global configuration vector
    v.updateRobotConfig(q_glob)
    embed()

def callback_forceRLEG(data):
    f = np.matrix(data.data).T
    M = v.robot.Mrf(q_glob)
    f_local = CONTACT_FORCE_ARROW_SCALE*f[:3]
    f_global = M.rotation * f_local
    p1 = M.translation
    p2 = p1+f_global
    v.moveArrow('Rf_force', p1, p2);

def callback_forceLLEG(data):
    f = np.matrix(data.data).T
    M = v.robot.Mlf(q_glob)
    f_local = CONTACT_FORCE_ARROW_SCALE*f[:3]
    f_global = M.rotation * f_local
    p1 = M.translation
    p2 = p1+f_global
    v.moveArrow('Lf_force', p1, p2);


def callback_floatingBase(data):
    global q_glob
    ff = np.array(data.data)
    ax,ay,az = ff[3:6]
    angle = np.linalg.norm(ff[3:6])
    if (angle > 1e-6):
        ax,ay,az = ff[3:6]/angle
        qx = ax * np.sin(angle/2)
        qy = ay * np.sin(angle/2)
        qz = az * np.sin(angle/2)
        qw = np.cos(angle/2)
    else:
        qx = 0.
        qy = 0.
        qz = 0.
        qw = 1.
    global ff_xzyquat 
    ff_xzyquat = np.hstack([ff[:3],np.array([qx,qy,qz,qw])])


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("robotState", Vector, callback_robotState)
    rospy.Subscriber("estimator_contactWrenchRightSole", Vector, callback_forceRLEG)
    rospy.Subscriber("estimator_contactWrenchLeftSole",  Vector, callback_forceLLEG)
    if READ_FREEFLYER_FROM_AA_SIGNAL:
        rospy.Subscriber("floatingBase_pos",  Vector, callback_floatingBase)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    #Run gepetto-viewer
    cmd = subprocess.Popen(['gepetto-viewer-server'])

    #set ROS Master URI
    os.environ["ROS_MASTER_URI"] = "http://hrp2014c:11311"
    
    #init display
    v = Viewer('hrp2',Hrp2014Wrapper(HRP2_URDF_MODEL, [MESH_PATH]))
    v.robot.viewer.gui.addFloor("world/floor")
    v.robot.viewer.gui.setLightingMode("world/floor","OFF")
    q = np.zeros(37)
    q[2]=0.8
    v.updateRobotConfig(q)

    f=np.matrix([0.,0.,1.]).T;
    p=np.matrix([2.,0.,0.]).T;
    
    # prepare 3d object
    #v.addSphere('zmpL',0.1,np.zeros(3));
    #v.addSphere('zmpR',0.1,np.zeros(3));
    v.addArrow('Rf_force', CONTACT_FORCE_ARROW_RADIUS, p, p+CONTACT_FORCE_ARROW_SCALE*f[:3], CONTACT_FORCE_ARROW_COLOR);
    v.addArrow('Lf_force', CONTACT_FORCE_ARROW_RADIUS, p, p+CONTACT_FORCE_ARROW_SCALE*f[:3], CONTACT_FORCE_ARROW_COLOR);
    v.robot.viewer.gui.setVisibility('world/Rf_force','ALWAYS_ON_TOP')
    v.robot.viewer.gui.setVisibility('world/Lf_force','ALWAYS_ON_TOP')
    
    listener()
