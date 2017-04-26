#!/usr/bin/env python

#Note: 
#  Install https://github.com/stack-of-tasks/dynamic_graph_bridge_msgs for ROS MSGs types
#  set ros master-ros to hrp2c: 
#    export ROS_MASTER_URI=http://hrp2014c:11311
DOWNSAMPLING = 1

from std_msgs.msg import String
from dynamic_graph_bridge_msgs.msg import Vector
from geometry_msgs.msg import TransformStamped
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
cpt_1 = 0
cpt_2 = 0
cpt_3 = 0
cpt_4 = 0
cpt_5 = 0
cpt_6 = 0
cpt_7 = 0
cpt_8 = 0

def callback_robotState(data):
    global cpt_1
    cpt_1 += 1;
    if (cpt_1%DOWNSAMPLING) :
        return
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
    #~ embed()
    #~ tau = se3.rnea(v.robot.model,v.robot.data,qUrdf,np.matlib.zeros(37),np.matlib.zeros(37))
    #~ print tau[-4]
    v.updateRobotConfig(q_glob)

def callback_forceRLEG(data):
    global cpt_2
    cpt_2+= 1;
    if (cpt_2%DOWNSAMPLING) :
        return
    f = np.matrix(data.data).T
    M = v.robot.Mrf(q_glob)
    f_local = CONTACT_FORCE_ARROW_SCALE*f[:3]
    f_global = M.rotation * f_local
    p1 = M.translation
    p2 = p1+f_global
    v.moveArrow('Rf_force', p1, p2);

def callback_forceLLEG(data):
    global cpt_3
    cpt_3+= 1;
    if (cpt_3%DOWNSAMPLING) :
        return
    f = np.matrix(data.data).T
    M = v.robot.Mlf(q_glob)
    f_local = CONTACT_FORCE_ARROW_SCALE*f[:3]
    f_global = M.rotation * f_local
    p1 = M.translation
    p2 = p1+f_global
    v.moveArrow('Lf_force', p1, p2);

def callback_CoM(data):
    global cpt_4
    cpt_4+= 1;
    if (cpt_4%DOWNSAMPLING) :
        return
    pt = np.matrix(data.data).T
    pt[2] = 0.
    v.updateObjectConfigRpy('CoM',pt)

def callback_CoP(data):
    global cpt_5
    cpt_5 += 1;
    if (cpt_5%DOWNSAMPLING) :
        return
    pt = np.matrix(data.data).T
    pt3d = np.matrix([pt[0,0], pt[1,0], 0.]).T;
    v.updateObjectConfigRpy('CoP',pt3d)

def callback_CoP_L(data):
    global cpt_6
    cpt_6 += 1;
    if (cpt_6%DOWNSAMPLING) :
        return
    pt = np.matrix(data.data).T
    v.updateObjectConfigRpy('CoP_L',pt)

def callback_CoP_R(data):
    global cpt_7
    cpt_7 += 1;
    if (cpt_7%DOWNSAMPLING) :
        return
    pt = np.matrix(data.data).T
    v.updateObjectConfigRpy('CoP_R',pt)


def callback_floatingBase(data):
    global cpt_8
    cpt_8 += 1;
    if (cpt_8%DOWNSAMPLING) :
        return
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

def callback_mocap_chest(data):
    config = se3.utils.XYZQUATToViewerConfiguration([[data.transform.translation.x,
                                                      data.transform.translation.y,
                                                      data.transform.translation.z,
                                                      data.transform.rotation.x,
                                                      data.transform.rotation.y,
                                                      data.transform.rotation.z,
                                                      data.transform.rotation.w]])
    v.updateObjectConfig('mocap_chest', config)


def listener():
    rospy.init_node('listener', anonymous=True)
    
    # robot configuration freeflyer in RPY + joints angles
    print 'subscribing...'
    rospy.Subscriber("robotState", Vector, callback_robotState)

    # Feet forces
    if DISPLAY_FEET_FORCES:
        rospy.Subscriber(TOPIC_FORCES_RF, Vector, callback_forceRLEG)
        rospy.Subscriber(TOPIC_FORCES_LF,  Vector, callback_forceLLEG)
    
    #optional freeflyer in angle axis
    if READ_FREEFLYER_FROM_AA_SIGNAL:
        rospy.Subscriber(TOPIC_FF,  Vector, callback_floatingBase)

    #CoPs
    if DISPLAY_COP_RL: 
        rospy.Subscriber(TOPIC_CoP_R, Vector, callback_CoP_R)
        rospy.Subscriber(TOPIC_CoP_L, Vector, callback_CoP_L)
    if DISPLAY_COP: 
        rospy.Subscriber(TOPIC_CoP,   Vector, callback_CoP  )

    #CoM
    if DISPLAY_COM: 
        rospy.Subscriber(TOPIC_CoM,   Vector, callback_CoM  )
    #chest Mocap
    if DISPLAY_MOCAP_CHEST :
        rospy.Subscriber(TOPIC_MOCAP_CHEST,   TransformStamped, callback_mocap_chest  )
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    #Run gepetto-viewer
    cmd = subprocess.Popen(['gepetto-viewer-server'])

    #set ROS Master URI
    #~ os.environ["ROS_MASTER_URI"] = "http://hrp2014c:11311"
    
    #init display
    v = Viewer('hrp2',Hrp2014Wrapper(HRP2_URDF_MODEL, [MESH_PATH]))
    v.robot.viewer.gui.addFloor("world/floor")
    v.robot.viewer.gui.setLightingMode("world/floor","OFF")
    q = np.zeros(37)
    q[2]=0.8
    v.updateRobotConfig(q)

    f=np.matrix([0.,0.,1.]).T;
    p=np.matrix([2.,0.,0.]).T;
    
    # prepare 3d objects
    if DISPLAY_COP_RL:
        v.addSphere('CoP_R',0.01,np.zeros(3),np.zeros(3),(1,0,0,1.0));
        v.addSphere('CoP_L',0.01,np.zeros(3),np.zeros(3),(1,0,0,1.0));
    if DISPLAY_COP:
        v.addSphere('CoP'  ,0.01,np.zeros(3),np.zeros(3),(1,0,0,1.0));
    if DISPLAY_COM:
        v.addSphere('CoM'  ,0.01,np.zeros(3),np.zeros(3),(0,1,0,1.0));
    
    if DISPLAY_FEET_FORCES :
        v.addArrow('Rf_force', CONTACT_FORCE_ARROW_RADIUS, p, p+CONTACT_FORCE_ARROW_SCALE*f[:3], CONTACT_FORCE_ARROW_COLOR);
        v.addArrow('Lf_force', CONTACT_FORCE_ARROW_RADIUS, p, p+CONTACT_FORCE_ARROW_SCALE*f[:3], CONTACT_FORCE_ARROW_COLOR);
        v.robot.viewer.gui.setVisibility('world/Rf_force','ALWAYS_ON_TOP')
        v.robot.viewer.gui.setVisibility('world/Lf_force','ALWAYS_ON_TOP')
    if DISPLAY_MOCAP_CHEST :
        v.robot.viewer.gui.addMesh('world/mocap_chest', v.robot.visual_model.geometryObjects[2].meshPath)
    listener()
