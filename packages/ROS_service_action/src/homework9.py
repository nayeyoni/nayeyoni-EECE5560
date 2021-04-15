#! /usr/bin/env python3

import rospy
from example_service.srv import Fibonacci, FibonacciResponse
import actionlib
import example_action_server.msg

def service(num):
    rospy.wait_for_service('calc_fibonacci')
    calc_fibonacci = rospy.ServiceProxy('calc_fibonacci', Fibonacci)
    rospy.loginfo(rospy.get_caller_id() + " Service starts with order %s", num)
    resp1 = calc_fibonacci(num)
    rospy.loginfo(rospy.get_caller_id() + " Service ends with order %s and result is %s", num, resp1)

def client(num):
    client = actionlib.SimpleActionClient('fibonacci', example_action_server.msg.FibonacciAction)
    client.wait_for_server()
    rospy.loginfo(rospy.get_caller_id() + " Action server Creating goal with order %s", num)
    goal = example_action_server.msg.FibonacciGoal(order=num)
    rospy.loginfo(rospy.get_caller_id() + " Action server Sending goal with order %s", num)
    client.send_goal(goal)
    rospy.loginfo(rospy.get_caller_id() + " Action server Waiting for result with order %s", num)
    client.wait_for_result()
    rospy.loginfo(rospy.get_caller_id() + " Action client Result with order %s: %s", num, client.get_result() )
    return client.get_result()  

if __name__=="__main__":
    rospy.init_node("homework9", anonymous=True)
    service(3)
    client(3)
    service(15)
    client(15)
    rospy.spin()
