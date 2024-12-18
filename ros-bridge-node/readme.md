# ROS2 Serial Bridge kk:

This project is a cpp implementation of a simple serial bridge between ROS2 Iron on a Linux system and
a micro controller like a Raspberrypi Pico.

I decided to go ahead with this project after experimenting with microROS on a Pi Pico.

While my experiments with microROS where reasonably successful in that I was successful 
in getting the typical sample publishers and subscribers working on the Pico, taking 
it farther in my project became difficult. 

Difficult how:

-   ROS is a large beast with some decent documentation at the introductory level but one soon finds 
    oneself reading the code to find how to do things and what can be done. 
    
-   In addition by running micro-ros I lost the ability so simple open the Arduino IDE and use the serial monitor
    to drive my Pi Pico. Also I lost the ability to simply put in a few `printf()`s 
    when trying to understand why something was misbehaving.

So I decided to implement a serial link based around lines of text (and a small amount of json encoding)
which meant I could stil use a serial terminal program like the Arduino Serial Monitor and drop in `printf()`
when I felt the need. 

However as you might have guessed this may have saved me from having to spend time reading the micro-ros
code but it did not save me from the ROS linux code base. I have spent many hours reading `rcl`, `rclcpp`, a bit of `rmw`
and interestingly the c/c++ code underneath `rclpy`.

# Project Status

-   This is still a work in progress.
-   This project requires companion firmware for a micro-controller, see [github.com/robertblackwell/pico-bridge-firmware](github.com/robertblackwell/pico-bridge-firmware)
-   current working components:
    
    - __bridge.cpp__  this is the actual bridge program in a form that is approaching final. It consists of two threads 
      - one running code that communicates with the micro controller 
      - one that is a ROS2 node with a number of __subscribers__ and __publishers__.
      - they (the two threads) communicate through 2 __threadsafe queues__
        - the 'input queue' allows the comms thread to pass 'messages' to the ROS Node thread. It wakes the Node thread using the __rclcpp::GuardCondition::trigger__ function.
        - the 'output queue' allows the ROS Node thread to pass 'messages' to the comms thread. It wakes the Comms thread using either:
          - a Linux __eventfd__ ,
          - or an equivalent well known trick with pipes. 
        
            Note the Comms thread only ever suspends/waits on a __select__ or __poll__ system call.
    
    - __publisher_test_cmds__ - is a standard ROS2 node that listens for ALL the messages that __bridge.cpp__ forwards from the micro controller. And also periodically sends 'command messages' to the micro controller.

    These two components constitute a __proof of concept__. 
    
    The ROS messages that these two components exchange are custom for this purpose and are dependencies for this project. They can be found
    at [https://gitbub.com/robertblackwell/ros2_bridge_msgs](https://gitbub.com/robertblackwell/ros2_bridge_msgs)

    The specific firmware required for the proof of concept is called (not surprisingly) __bridge.cpp__. 


-   other components:

    -   __bridge_speed_test.cpp__ together with the correct firware (__test_transport.cpp__) perform a communication speed test. The firmware sends data as fast as it can and the Linux program (`bridge_speed_test.cpp`) reads, optionally displays it, and computes throughput. I was surprised to see numbers in the range of 660KBytes/Sec to 690KBytes/sec.
    -  __bridg_test_cmds.cpp__ like __bridge.cpp__ consists of a comms thread and a ROS Node thread. The ROS Node cycles through the full vocabulary of messages to demonstrate the micro controller is seeing them and responding. The Verificaton is at this point only me watching the output for anomolies.  


# Whats next

-   a few more communication speed tests - both ways at the "same" time.
-   running all of this on a RaspberryPi so that I can start it talking to my robot while it moves. This 'bridge' thing is really just a rabbit hole I fell down while trying to  get a simple robot working.
-   hook up a ROS2 Node being driven by a game controller.
-   have my robot publish `odometry` messages so I can start working with ROS2 navigation.
