# RoboTag
Our Team: Lucian - lfairbrother@brandeis.edu, Eyal - eyalcohen@brandeis.edu, Chris - mchoi@brandeis.edu

# Final Project, Our Demo:
For our final project we want to have atleast three robots participating
in a game of interactive tag. In this game each robot will be switching
off roles when it is "Tagged". We haven't decided 100% on what being
tagged entails but for right now we will say a robot is tagged when it 
is x distance away from the tagger. Our robots will be able to play this
in an environment with other objects, such that they must navigate. 

# Learning Goals
As a group we hope to be able to understand pathfinding and using this with
a moving destination ie. a robot running away. We hope to learn how to direct 
a robot to find another robot that is broadcasting its location (tf2). 
We would like tocreate a project that is very flexible. Over the weeks as we work
on it, we start with the Hello World version of our project and switch out each 
aspect of the program with a more advanced and in depth algorithym. For example 
improve our run away algorithym, and then improve our pathfinding chasing algorithym and so on.

# Evaluation
We would like to be evaluated on your percieved group effort, the functionality
of the cop/chasing robot in finding the robber/runner robot.


roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/tagmap.yaml

ssh ubuntu@100.86.78.102
