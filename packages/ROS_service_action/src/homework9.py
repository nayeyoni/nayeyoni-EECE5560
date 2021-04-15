#! /usr/bin/env python3

import rospy
from example_service.srv import Fibonacci, FibonacciResponse
import actionlib
import example_action_server.msg


if __name__ == "__main__":
    try: 
        rospy.init_node('homework9')
    
        client = actionlib.SimpleActionClient('fibonacci', example_action_server.msg.FibonacciAction)
        client.wait_for_server()
      
        goal = example_action_server.msg.FibonacciGoal(order=15)

        client.send_goal(goal)
        client.wait_for_result()

        result = client.get_result()
        print("Raa")
        print("Result:", ', '.join([str(n) for n in result.sequence])) 
    except rospy.ROSInterruptException:
        pass
