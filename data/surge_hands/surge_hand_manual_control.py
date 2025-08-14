
import math

import pybullet as p
import time

import pybullet_data

use_real_hardware = False
if use_real_hardware:
  import serial

  # Configure the serial connection
  ser = serial.Serial(
      port='/dev/tty.usbserial-AU05J61K', #/dev/ttyUSB0',"COM8",    # Change this if your device is on a different port
      baudrate=115200,        # Set the baud rate
      bytesize=serial.EIGHTBITS,
      parity=serial.PARITY_NONE,
      stopbits=serial.STOPBITS_ONE,
      timeout=1               # Timeout in seconds
  )


#p.connect(p.GUI, options="--background_color_red=1 --background_color_blue=1 --background_color_green=1")
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

#collision tweak
#p.setPhysicsEngineParameter(enableSAT=1)
p.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=128, cameraPitch=-28, cameraTargetPosition=[-0.5,-0.4,-0.3])

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)


flags = p.URDF_INITIALIZE_SAT_FEATURES

allow_self_collision = True
if allow_self_collision:
	flags+= p.URDF_USE_SELF_COLLISION

robot = p.loadURDF("surge_v13_hand_right_textured_pybullet.urdf", useMaximalCoordinates=False, useFixedBase=True, flags=flags)
#robot = p.loadURDF("surge_v13_hand_right_pybullet.urdf", useMaximalCoordinates=False, useFixedBase=True, flags=flags)
#robot = p.loadURDF("surge_v13_hand_left_textured_pybullet.urdf", useMaximalCoordinates=False, useFixedBase=True, flags=flags)
#robot = p.loadURDF("surge_v13_hand_left_pybullet.urdf", useMaximalCoordinates=False, useFixedBase=True, flags=flags)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
jointIds = []
paramIds = []

p.setPhysicsEngineParameter(numSolverIterations=10)
p.changeDynamics(robot, -1, linearDamping=0, angularDamping=0)

joint_name_to_hw={}
joint_name_to_hw[b"Index_MCP_Joint"]='I'
joint_name_to_hw[b"Middle_MCP_Joint"]='M'
joint_name_to_hw[b"Ring_MCP_Joint"]='R'
joint_name_to_hw[b"Pinky_MCP_Joint"]='P'
joint_name_to_hw[b"Metacarpal_Joint"]='X'
joint_name_to_hw[b"Thumb_Joint"]='T'

joint_name_to_hw_scaling={}
joint_name_to_hw_scaling[b"Index_MCP_Joint"]=1.0
joint_name_to_hw_scaling[b"Middle_MCP_Joint"]=1.0
joint_name_to_hw_scaling[b"Ring_MCP_Joint"]=1.0
joint_name_to_hw_scaling[b"Pinky_MCP_Joint"]=1.0
joint_name_to_hw_scaling[b"Metacarpal_Joint"]=0.4
joint_name_to_hw_scaling[b"Thumb_Joint"]=0.3


joint_index_to_name={}
coupled_joint_index={}
coupled_child_joints = {}
coupled_child_joints[b"Index_PIP_Joint"]=-1
coupled_child_joints[b"Middle_PIP_Joint"]=-1
coupled_child_joints[b"Ring_PIP_Joint"]=-1
coupled_child_joints[b"Pinky_PIP_Joint"]=-1
coupled_parent_to_child={}

#disable all self-collision except finger tips and thumb

#only allow collision between finger tips and either thumb or palm
allowed_thumb_collisions = [
(b"Thumb_Joint",b"Index_PIP_Joint"),
(b"Thumb_Joint",b"Middle_PIP_Joint"),
(b"Thumb_Joint",b"Ring_PIP_Joint"),
(b"Thumb_Joint",b"Pinky_PIP_Joint"),
]

allowed_palm_collisions = [
b"Index_PIP_Joint",
b"Middle_PIP_Joint",
b"Ring_PIP_Joint",
b"Pinky_PIP_Joint",
]

#only enable allowed self-collisions
for second in range(p.getNumJoints(robot)):
  second_name = p.getJointInfo(robot, second)[1]
  if second_name in allowed_palm_collisions:
    print("enable", ("Palm",second_name))
    #index -1 is palm
    p.setCollisionFilterPair(robot, robot,-1, second, True)
    p.setCollisionFilterPair(robot, robot,second,-1, True)
  else:
  	p.setCollisionFilterPair(robot, robot,second,-1, False)
  	p.setCollisionFilterPair(robot, robot,-1,second, False)

for first in range(p.getNumJoints(robot)):
  first_name = p.getJointInfo(robot, first)[1]
  for second in range(p.getNumJoints(robot)):
    second_name = p.getJointInfo(robot, second)[1]
    print("first_name=", first_name, " second_name=", second_name)
    
    if (first_name,second_name) in allowed_thumb_collisions or (second_name,first_name) in allowed_thumb_collisions:
      print("enable", (first_name,second_name))
      p.setCollisionFilterPair(robot, robot,first, second, True)
    else:
      p.setCollisionFilterPair(robot, robot,first, second, False)




for j in range(p.getNumJoints(robot)):
  p.changeDynamics(robot, j, linearDamping=0, angularDamping=0)
  info = p.getJointInfo(robot, j)
  #print(info)
  jointName = info[1]
  jointType = info[2]
  if jointName  in joint_name_to_hw:
    print("jointName,", joint_name_to_hw[jointName])
    param_index = len(jointIds)
    joint_index_to_name[param_index ]=jointName
  if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
    if jointName in coupled_child_joints:
      parent_index = jointIds[len(jointIds)-1]
      coupled_child_joints[jointName]=parent_index
      coupled_parent_to_child[parent_index]=j
      pass
    else:
      jointIds.append(j)
      paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), 0, 1.92, 0))

print("coupled_child_joints=",coupled_child_joints)

while (1):
  for i in range(len(paramIds)):
    
    c = paramIds[i]
    try:
     targetPos = p.readUserDebugParameter(c)
     if i in joint_index_to_name:
      deg = int(math.degrees(targetPos))
      #print(joint_name_to_hw[joint_index_to_name[i]], deg)

      if use_real_hardware:
        # Send the command
        #TODO: do we need to a scaling factor for the thumb for real hardware?
        command = "AT+SJA="+str(deg*joint_name_to_hw_scaling[joint_index_to_name[i]])+","+joint_name_to_hw[joint_index_to_name[i]]+"\r\n"  # \r\n is typically required for AT commands
        
        #print("command=",command)
        ser.write(command.encode())  # Encode the string and send it

        # Read the response
        response = ser.read(ser.inWaiting())  # Read all available data
        #print("Response:", response.decode(errors='ignore'))  # Decode and print response
      
     finger_force= 100 #Newton
     p.setJointMotorControl2(robot, jointIds[i], p.POSITION_CONTROL, targetPos, force=finger_force)
     if jointIds[i] in coupled_parent_to_child:
      distal_to_proximal_scaling=0.65 #you can also use a position-dependent non-linear function here
      p.setJointMotorControl2(robot, coupled_parent_to_child[jointIds[i]], p.POSITION_CONTROL, targetPos*distal_to_proximal_scaling, force=finger_force)
    except(e):
      print(e)
 
  contacts = p.getContactPoints()
  num_contacts = len(contacts)
  if (num_contacts):
  	print("num_contacts=", num_contacts)
  p.stepSimulation() 
  time.sleep(1./240.)
