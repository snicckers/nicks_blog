---
layout: post
title: "Flight Controller: Part 1 Single Axis Controller"
author: "Nick"
---

<hr>

This blog series documents the development of a quadcopter flight controller. It is written for my own purposes, and I would not recommend using it as an educational resource. This part of the series focuses on the creation of a single axis stabiliser, with one and two motors, fixed to a test platform. 

Upon initial research I discovered the tutorial series by Joop Brokking. However I did not want to wholesale copy/paste his or others' code, rather I wanted to understand what I was building step by step, mostly for my own satisfaction rather than any practical purpose. I did however use his, and some other resources, as reference while working on this first part of the project. See below links to the resources used:

_Insert Links Here_

My overall plan for the project was as follows:
1. Build a controller that worked on a single fixed axis
2. Build a controller that worked in 3 degrees of freedom, fixed to a test stand
3. Test the controller with unrestricted flight

This post focuses on Step 1. Steps 2 & 3 will be addressed in the third post, and the second post will focus on instrumentation.

## Section 1 - First Steps

The instrumentation gets a lot more complicated later on, were Euler angles found by integrating gyro rates from an MPU6050. MEMS gyro rates always gradually drift, causing inaccuracies over long time periods. The solution is to correct the drift using accelerometer data using a complimentary filter.  This is a huge oversimplification, but as mentioned I will go into depth on instruments in the second blog post. There is no rotation matrix for the gyro angles, so no tilt compensation is performed in this IMU (aside from the accelerometer slowly correcting the angle), and it is not the final one that I went with, but for getting a controller to work on a single axis it does the job.

I built an admittedly terrible setup, and programmed a simple Proportional, Integral, & Derivative Controller (see next section) and tested it. It did awfully & in retrospect I should have realized this ahead of time. Here's a video: