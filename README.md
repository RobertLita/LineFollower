# Line Follower 

### This was the last project of Introduction to Robotics course: Line Follower. 

## Description 
 - A line follower project is a robot that is designed to follow a specific path or line 🚗. The robot uses sensors to detect the location of the line and then adjusts its movement accordingly. 
 - We used PID controller 🕹️ - a control loop feedback mechanism that keeps the robot on the line. It uses Proportional, Integral and Derivative to minimize the error. 

## Components 📌 - from a line follower kit
 - Arduino Uno
 - 2 DC motors
 - 2 wheels
 - LiPo Battery
 - QTR-8A reflectance sensor (8 IR array) -> out of which we used 6
 - L293D motor driver
 - wires, breadboard, chassis

 ## Features & Parameters ⚙️
 - We made an automatic calibration for 6 seconds by powering the right motor until the sensor is out of the line and after we power the left motor and so on.
 - Then we move the car left or right until the sensor is centered on the line.
 - We map the error from sensors to [-50, 50] interval and apply PID with kp = 40, ki = 0, kd = 4. 
 - For a better approach in tight turns, we spin one motor forward and the other backwards.

## Context 🚀
 - We got the line follower kit.
 - We assembled it at lab.
 - Need to code the PID Controller (in our case just PD) to make the robot complete a "mistery" track in less than 20 seconds.
 - The coding part was done during a 1 day hackathon.

## Results 🏎️
 - The best time of our robot was 25.839s (from 3 tries).
 - The robot keeps shaking on the line, maybe the kp was a little high.

### I teamed up with <a href = "https://github.com/CristiGitHub">Cristi Lefter</a> (Team Chelie).
<br>

 ![Line Follower image](assets/LineFollower.jpeg)

 <div align="center">
  <h3>
    <a href="https://youtu.be/RLEqACWAsrI">
      ▶️ Click here for video
    </a>
  </h3>
</div>


