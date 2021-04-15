#! /usr/bin/env python3

import rospy
from example_service.srv import Fibonacci, FibonacciResponse
import actionlib
import example_action_server.msg

def service(num):
    rospy.wait_for_service('calc_fibonacci')
    calc_fibonacci = rospy.ServiceProxy('calc_fibonacci', Fibonacci)
    rospy.loginfo(rospy.get_caller_id() + "Order %s: Service starts", num)
    resp1 = calc_fibonacci(num)
    rospy.loginfo(rospy.get_caller_id() + "Order %s: Service ends and sequence is %s", num, resp1)

def client(order):
    client = actionlib.SimpleActionClient('fibonacci', example_action_server.msg.FibonacciAction)
    client.wait_for_server()
    rospy.loginfo(rospy.get_caller_id() + "Sending goal with order %s", num)
    goal = example_action_server.msg.FibonacciGoal(order=num)
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo(rospy.get_caller_id() + "Result with order %s: %s", num, client.get_result() )
    return client.get_result()  

if __name__=="__main__":
    rospy.init_node("homework9", anonymous=True)
    service(3)
    client(3)
    service(15)
    client(15)
    rospy.spin()

