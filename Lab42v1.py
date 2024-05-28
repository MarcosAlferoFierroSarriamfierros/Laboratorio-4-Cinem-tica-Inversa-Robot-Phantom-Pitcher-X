# -*- coding: utf-8 -*-
import math
import rospy
from std_srvs.srv import Empty
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pynput.keyboard import Key, Listener, KeyCode  


joint_limits = {
    "joint_1": (-math.radians(130), math.radians(90)),
    "joint_2": (-math.radians(45), math.radians(45)),
    "joint_3": (-math.radians(155), math.radians(155)),
    "joint_4": (-math.radians(90), math.radians(90)),
    "tool": (-math.radians(90), math.radians(90))
}

def are_positions_within_limits(positions):
    joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "tool"]
    for i, position in enumerate(positions):
        joint_name = joint_names[i]
        lower_limit, upper_limit = joint_limits[joint_name]
        if not (lower_limit <= position <= upper_limit):
            return False, joint_name
    return True, None

def callback(data):
    print(f"Joint1: {round(math.degrees(data.position[0]),2)} Joint2: {round(math.degrees(data.position[1]),2)} Joint3: {round(math.degrees(data.position[2]),2)} Joint4: {round(math.degrees(data.position[3]),2)} Joint5: {round(math.degrees(data.position[4]),2)}")
    rospy.sleep(3)

def listener():
    rospy.Subscriber("/dynamixel_workbench/joint_states", JointState, callback)
    #rospy.spin()

def joint_publisher():
    pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)
    rospy.init_node('joint_publisher', anonymous=False)
    
    # Print de comandos
    welcome = """\n 
                  
Para posicionar el robot, presione la tecla correspondiente y luego Enter:

            1:  Posición Home   (0, 0, 0, 0, 0.)
            2:  Posición 2     (25, 25, 20, -20, 0.)
            3:  Posición 3     (-35,35, -30, 30, 0.)
            4:  Posición 4     (85, -20, 55, 25, 0.)
            5:  Posición 5     (80, -35, 55, 45, 0)

                  """
    rospy.loginfo(welcome)
    while not rospy.is_shutdown():
        key = input()
        
        positions_dict = {
            '1': [math.radians(0), math.radians(0), math.radians(0), math.radians(0), math.radians(0)],
            '2': [math.radians(45), math.radians(25), math.radians(20), math.radians(-20), math.radians(0)],
            '3': [math.radians(-15), math.radians(35), math.radians(-30), math.radians(30), math.radians(0)],
            '4': [math.radians(85), math.radians(-10), math.radians(55), math.radians(25), math.radians(0)],
            '5': [math.radians(90), math.radians(35), math.radians(75), math.radians(-45), math.radians(0)],
            "6":[math.radians(-365), math.radians(200), math.radians(590), math.radians(700), math.radians(1000)],
            '9': [-0.3, 2.2, -2.5, -1.3, 0.3]
        }


        if key in positions_dict:
            positions = positions_dict[key]
            within_limits, joint_name = are_positions_within_limits(positions)
            
            if within_limits:
                state = JointTrajectory()
                state.header.stamp = rospy.Time.now()
                state.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "tool"]
                point = JointTrajectoryPoint()
                point.positions = positions
                point.time_from_start = rospy.Duration(0.5)
                state.points.append(point)
                pub.publish(state)
                rospy.loginfo(f'Posición {key} enviada')
            else:
                rospy.logwarn(f'Posición {key} no válida para la articulación {joint_name}')
            rospy.sleep(1)
        
        elif key == '0':
            listener()

if __name__ == '__main__':
    try:
        joint_publisher()
    except rospy.ROSInterruptException:
        pass
