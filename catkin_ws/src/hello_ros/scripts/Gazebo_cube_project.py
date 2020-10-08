#!/usr/bin/env python
import roslib
roslib.load_manifest('hello_ros')
 
import sys
import copy
import rospy
import tf_conversions
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg as shape_msgs
import math
 
from std_msgs.msg import String


from sensor_msgs.msg import JointState
from numpy import zeros, array, linspace
from math import ceil
from gazebo_msgs.srv import GetModelState


def init_pose():
  print "============ Go Init Pose"
  initial_pose = group.get_current_pose().pose
  waypoints = []

  waypoints.append(initial_pose)

  initial_pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0,  -math.pi/2  , 0.0))
  
  initial_pose.position.x =0.40
  initial_pose.position.y =-0.10
  initial_pose.position.z =1.35

  #Create waypoints
  waypoints.append(initial_pose)
 
  #createcartesian  plan
  (plan1, fraction) = group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold
  #plan1 = group.retime_trajectory(robot.get_current_state(), plan1, 1.0)

  rospy.sleep(2)

  ## Moving to a pose goal
  group.execute(plan1,wait=True)
  rospy.sleep(4.)

def go_down():
  print "============ Go Down"
  downwards_pose = group.get_current_pose().pose
  waypoints = []

  waypoints.append(downwards_pose)

  downwards_pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0,  -math.pi/2  , 0.0))
  
  #initial_pose.position.x =0.40
  #initial_pose.position.y =-0.10
  downwards_pose.position.z = 0.8

  #Create waypoints
  waypoints.append(downwards_pose)
 
  #createcartesian  plan
  (plan1, fraction) = group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold
  #plan1 = group.retime_trajectory(robot.get_current_state(), plan1, 1.0)

  rospy.sleep(2)

  ## Moving to a pose goal
  group.execute(plan1,wait=True)
  rospy.sleep(2.)

def go_up():
  print "============ Go Up"
  upwards_pose = group.get_current_pose().pose
  waypoints = []

  waypoints.append(upwards_pose)

  upwards_pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0,  -math.pi/2  , 0.0))
  
  #initial_pose.position.x =0.40
  #initial_pose.position.y =-0.10
  upwards_pose.position.z = 1.45

  #Create waypoints
  waypoints.append(upwards_pose)
 
  #createcartesian  plan
  (plan1, fraction) = group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold
  #plan1 = group.retime_trajectory(robot.get_current_state(), plan1, 1.0)

  rospy.sleep(2)

  ## Moving to a pose goal
  group.execute(plan1,wait=True)
  rospy.sleep(2.)

def bucket_pose():
  print "============ Go to Bucket"
  # x=0.53, y=-0.23
  bucket_pose = group.get_current_pose().pose
  waypoints = []

  waypoints.append(bucket_pose)

  bucket_pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0,  -math.pi/2  , 0.0))
  bucket_pose.position.x =0.53
  bucket_pose.position.y =-0.23
  bucket_pose.position.z =1.2



  #Create waypoints
  waypoints.append(bucket_pose)
 
  #createcartesian  plan
  (plan1, fraction) = group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold
  #plan1 = group.retime_trajectory(robot.get_current_state(), plan1, 1.0)

  rospy.sleep(2)

  ## Moving to a pose goal
  group.execute(plan1,wait=True)
  rospy.sleep(4.)

def get_cube_location(i):
  try:
      cube_location = []
      model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
      blockName = 'cube{}'.format(i)
      resp_coordinates = model_coordinates(blockName,'link')
      print '\n'
      print 'Status.success = ', resp_coordinates.success
      if resp_coordinates.success == True:
        print(blockName)
        print("Cube " + str(blockName))
        cube_location.append(resp_coordinates.pose.position.x)
        cube_location.append(resp_coordinates.pose.position.y)
        cube_location.append(resp_coordinates.pose.position.z)
        print("Value of X : " + str(resp_coordinates.pose.position.x))
        print("Value of Y : " + str(resp_coordinates.pose.position.y)) 
        print("Value of Z : " + str(resp_coordinates.pose.position.z))

      else:
        print('No more cubes')
  except rospy.ServiceException as e:
      rospy.loginfo("Get Model State service call failed:  {0}".format(e))
  return cube_location


def cube_pose(cube_location,i):
  print('=========Go to Cube{}'.format(i))
  print(cube_location)
  cube_pose = group.get_current_pose().pose
  waypoints = []

  waypoints.append(cube_pose)

  cube_pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.0,  -math.pi/2  , 0.0))
  cube_pose.position.x =cube_location[0]
  cube_pose.position.y =cube_location[1]
  #cube_pose.position.z =cube_location[2]



  #Create waypoints
  waypoints.append(cube_pose)
 
  #createcartesian  plan
  (plan1, fraction) = group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold
  #plan1 = group.retime_trajectory(robot.get_current_state(), plan1, 1.0)

  rospy.sleep(2)

  ## Moving to a pose goal
  group.execute(plan1,wait=True)
  rospy.sleep(4.)
 

def end_programm():
  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()
 
  ## END_TUTORIAL
  print "============ STOPPING"
  R = rospy.Rate(10)
  while not rospy.is_shutdown():
    R.sleep()


def jointStatesCallback(msg):
  currentJointState = JointState()
  global currentJointState
  currentJointState = msg


def close_gripper():
  print('========= Closing Gripper')
  currentJointState = JointState()
  #rospy.init_node('test_publish')
 
  # Setup subscriber
  #rospy.Subscriber("/joint_states", JointState, jointStatesCallback)
 
  pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1)
 
  currentJointState = rospy.wait_for_message("/joint_states",JointState)
  print 'Received!'
  currentJointState.header.stamp = rospy.get_rostime()
  tmp = 0.7
  #tmp_tuple=tuple([tmp] + list(currentJointState.position[1:]))
  currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp])
  rate = rospy.Rate(10) # 10hz
  for i in range(3):
    pub.publish(currentJointState)
    print 'Published!'
    rospy.sleep(2.)
 
  print 'end!'


def open_gripper():
  print('=========== Opening Gripper')
  currentJointState = JointState()
  
  #rospy.init_node('test_publish')
 
  # Setup subscriber
  #rospy.Subscriber("/joint_states", JointState, jointStatesCallback)
 
  pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1)
 
  currentJointState = rospy.wait_for_message("/joint_states",JointState)
  print 'Received!'
  currentJointState.header.stamp = rospy.get_rostime()
  tmp = 0.005
  #tmp_tuple=tuple([tmp] + list(currentJointState.position[1:]))
  currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp])
  rate = rospy.Rate(10) # 10hz
  for i in range(3):
    pub.publish(currentJointState)
    print 'Published!'
    rospy.sleep(2.)
 
  print 'end!'

 
def move_group_python_interface_tutorial():
  ## BEGIN_TUTORIAL
  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)
 
  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group = moveit_commander.MoveGroupCommander("Arm")
 
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)
 
  print "============ Starting tutorial "
  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group.get_planning_frame()
  ## We can also print the name of the end-effector link for this group
  print "============ End effector frame: %s" % group.get_end_effector_link()
  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()
  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"
 
  ## Let's setup the planner
  #group.set_planning_time(0.0)
  group.set_goal_orientation_tolerance(0.01)
  group.set_goal_tolerance(0.01)
  group.set_goal_joint_tolerance(0.01)
  group.set_num_planning_attempts(100)
 
  ## Planning to a Pose goal
  print "============ Generating plan 1"
 
  pose_goal = group.get_current_pose().pose
  waypoints = []
 
  #pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.,  0.  , 0.))
  waypoints.append(pose_goal)
  pose_goal.position.x =0.40
  pose_goal.position.y =-0.10
  pose_goal.position.z =1.55
  print pose_goal
 
  #Create waypoints
  waypoints.append(pose_goal)
 
  #createcartesian  plan
  (plan1, fraction) = group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold
  #plan1 = group.retime_trajectory(robot.get_current_state(), plan1, 1.0)
 
 
  print "============ Waiting while RVIZ displays plan1..."
  rospy.sleep(0.5)
 
 
  # ## You can ask RVIZ to visualize a plan (aka trajectory) for you.
  # print "============ Visualizing plan1"
  # display_trajectory = moveit_msgs.msg.DisplayTrajectory()
  # display_trajectory.trajectory_start = robot.get_current_state()
  # display_trajectory.trajectory.append(plan1)
  # display_trajectory_publisher.publish(display_trajectory);
  # print "============ Waiting while plan1 is visualized (again)..."
  # rospy.sleep(2.)
 
  #If we're coming from another script we might want to remove the objects
  if "table" in scene.get_known_object_names():
    scene.remove_world_object("table")
  if "table2" in scene.get_known_object_names():
    scene.remove_world_object("table2")
  if "groundplane" in scene.get_known_object_names():
    scene.remove_world_object("groundplane")
 
  ## Moving to a pose goal
  group.execute(plan1,wait=True)
  rospy.sleep(4.)
  ## second movement
  pose_goal2 = group.get_current_pose().pose
  waypoints2 = []
  #pose_goal.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0.,  0.  , 0.))
  waypoints.append(pose_goal)
  pose_goal2.position.x =0.40
  pose_goal2.position.y =-0.10
  pose_goal2.position.z =1.2
  print pose_goal2
 
  #Create waypoints
  waypoints2.append(copy.deepcopy(pose_goal2))
 
  #createcartesian  plan
  (plan2, fraction) = group.compute_cartesian_path(
                                      waypoints2,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold
  #plan1 = group.retime_trajectory(robot.get_current_state(), plan1, 1.0)
  rospy.sleep(2.)
 
 # ## You can ask RVIZ to visualize a plan (aka trajectory) for you.
 #  display_trajectory = moveit_msgs.msg.DisplayTrajectory()
 #  display_trajectory.trajectory_start = robot.get_current_state()
 #  display_trajectory.trajectory.append(plan2)
 #  display_trajectory_publisher.publish(display_trajectory);
 
 #  rospy.sleep(2)
 
  group.execute(plan2,wait=True)
  rospy.sleep(2.)
 
  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()
 
  ## END_TUTORIAL
  print "============ STOPPING"
  R = rospy.Rate(10)
  while not rospy.is_shutdown():
    R.sleep()

#--------------------------------------------
#--------------------------------------------
#               MAIN
#--------------------------------------------
#--------------------------------------------
if __name__=='__main__':

#--------------------------------------------
# Setup
#--------------------------------------------
  ## First initialize moveit_commander and rospy.
  print("============ Starting tutorial setup")
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group = moveit_commander.MoveGroupCommander("Arm")

  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)
  print "============ Reference frame: %s" % group.get_planning_frame()
  ## We can also print the name of the end-effector link for this group
  print "============ End effector frame: %s" % group.get_end_effector_link()
  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()
  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"

  ## Let's setup the planner
  #group.set_planning_time(0.0)
  group.set_goal_orientation_tolerance(0.01)
  group.set_goal_tolerance(0.01)
  group.set_goal_joint_tolerance(0.01)
  group.set_num_planning_attempts(100)

#---------------------------------------------------------------
# Define Init Pose
#---------------------------------------------------------------

  try:
    
    
    init_pose()
    for cube_number in range (0,6):
      cube_location = get_cube_location(cube_number)
      
      cube_pose(cube_location,cube_number)
      go_down()
      close_gripper()
      
      go_up()
      bucket_pose()
      open_gripper()

    end_programm()
   

  except rospy.ROSInterruptException:
    print('============EXEPTION')
    pass