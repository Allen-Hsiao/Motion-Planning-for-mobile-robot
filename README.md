# Motion-Planning-for-mobile-robot

https://user-images.githubusercontent.com/67730218/125384858-1cbe5000-e3cc-11eb-8196-f71be21c1485.mov

# Problem Statement
The task is to navigate in a real world environment without colliding with obstacles and finally kick the ball to the gate.
![image](https://user-images.githubusercontent.com/67730218/125385395-fe0c8900-e3cc-11eb-8596-198fd32f3716.png)



# Design Idea
To achieve the task, we follow the instruction step by step.
1. Setting the real world environment into a grid map.
2. Building the relation between robot position and map.
3. Setting all parameter in the environment.
4. Introduce A* algorithm and generate polynomial trajectories.
5. Test and verify PID controller.
6. Simulation

# Results (Figure from Problem Statement)
The middle path figure is the result of simulation in Gazebot. We successfully have robot navigate from start point to goal point without any collision. Also, the right figure shows the robot demoing in the real environment and the path achieved the path we expected.

# Debugging Issue
1. Given that the grid map is 0.5m each side, we forget to change distance between neighbor point from 1m to 0.5m. Thus, the A* algorithm    couldn’t return the optimal path.
2. To prevent from hitting the two obstacles and the wall, we focused on the path curve. In the PID controller, the coefficient Kp and Kd affect the adjustment of angular velocity. We trial and error many times to find the appropriate number.
3. In the demo, robot was placed at random grid of the green area. We have to set exactly x and y position for star point. Initially, we misunderstood the robot position. After that, we reviewed the environment setting and figured out where is the reference position in real environment finally.
4. Simulating in Gazebot is slightly different to the demo. After failing a couple times, we figured out there were some conditions might be the root cause. First, the real robot size is much bigger than we expected compare to the model in Gazebot. It caused the robot hit the obstacle. Secondly, the real robot’s parameters are slightly different from model such as speed, angular velocity. The path was also slightly different from the path in simulation. Thus. we applied different PID parameters to achieve the task.
