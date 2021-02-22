#!/usr/bin/env python

import rospy

# import our new fizzbuzz message type
from fizzbuzz.msg import FizzBuzz 
from std_msgs.msg import Int32

class FizzBuzzNode:
    def __init__(self):
        rospy.loginfo("Starting fizzbuzz_node")

        self.total_numbers = 0
        self.total_fizz = 0
        self.total_buzz = 0
        self.total_fizzbuzz = 0

        # create a publisher object to send data
        self.fizzbuzz_pub = rospy.Publisher("fizzbuzz_stats", FizzBuzz, queue_size=10)

        # subscribe to the numbers topic
        rospy.Subscriber("numbers", Int32, self.number_callback)

    def number_callback(self, msg):
        # this function is called whenever a number is recived.

        number = msg.data

        fizzbuzz_str = self.fizzbuzz(number)
        # loginfo to print the string to the terminal
        rospy.loginfo(fizzbuzz_str)

        fizzbuzz_msg = FizzBuzz()
        fizzbuzz_msg.fizzbuzz = fizzbuzz_str
        fizzbuzz_msg.fizz_ratio = float(self.total_fizz) / float(self.total_numbers)
        fizzbuzz_msg.buzz_ratio = float(self.total_buzz) / float(self.total_numbers)
        fizzbuzz_msg.fizzbuzz_ratio = float(self.total_fizzbuzz) / float(self.total_numbers)
        fizzbuzz_msg.number_total = self.total_numbers

        # publish the message
        self.fizzbuzz_pub.publish(fizzbuzz_msg)

    def fizzbuzz(self, number):
        # This should return a string equal to:
        #      "fizz" if number divisible my 3
        #      "buzz" if number divisible my 5
        #      "fizzbuzz" if number divisible my 15
        #      a string containing the number otherwise

        result = str(number)
        self.total_numbers += 1

        if number % 15 == 0:
            result = "fizzbuzz"
            self.total_fizzbuzz += 1
        elif number % 5 == 0:
            result = "buzz"
            self.total_buzz += 1
        elif number % 3 == 0:
            result = "fizz"
            self.total_fizz += 1

        return result


if __name__ == "__main__":
    rospy.init_node("fizzbuzz_node")
    node = FizzBuzzNode()
    rospy.spin()
