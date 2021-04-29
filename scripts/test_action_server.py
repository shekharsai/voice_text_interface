#! /usr/bin/env python

import rospy
import actionlib 

from voice_text_interface.msg import WashTheDishesAction, WashTheDishesFeedback, WashTheDishesResult

class ActionServer():
    def __init__(self):
        self.a_server = actionlib.SimpleActionServer("wash_dish", WashTheDishesAction, execute_cb=self.execute_cb, auto_start=False)
        self.a_server.start()

    def execute_cb(self, goal):
        success = True
        last_dish_washed = ''        
        dishes_washed = []
        feedback = WashTheDishesFeedback()
        result = WashTheDishesResult()
        rate = rospy.Rate(2)

        for i in range(0, goal.number_of_minutes):

             if self.a_server.is_preempt_requested():
                 success = False
                 break

             last_dish_washed = "bowl-" + str(i)
             feedback.last_dish_washed = last_dish_washed
             result.dishes_washed.append(last_dish_washed)
             self.a_server.publish_feedback(feedback)
             rate.sleep()
        
        if success:
            self.a_server.set_succeeded(result)     

if __name__ == "__main__":
    rospy.init_node("WashDishActionServer")
    s = ActionServer()
    rospy.spin()

