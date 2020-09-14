---
usemathjax: true
layout: post
title: "HMI/SCADA Project Demo"
author: "Nick"
---

Over the course of a few weeks in lockdown I completed several PLC courses, and I'm proud enough of the results of one project to show it off. 

A PLC program running in RSLogix 500 is provided, and you are expected to create a functioning SCADA (Supervisory Control And Data Aquisition) system that provides full control of the system alongside data logging features. 

The RSLogix 500 file includes simulation for the system, which is a water filtration system for a tank. It allows for filling of the tank when the water level is too low and cleaning of the filter if it becomes clogged (backwash cycle) - triggered when the pressure across the filter exceeds a setpoint. 

While it comes fully featured, I added a few rungs that set the relevant bit when certain pipes should have water flowing through them, and a general alarm bit that goes off if any of the alarms are triggered.

### Demonstration

The project called for the SCADA system to be implemented using a range of different software, all of them available under trial or educational licenses. Here I'm showing off my best & last attempt, made using Indusoft Web Studio, with the following screens:

#### 1. System Overview
Graphical overview of the system. My goal was to only show what an operator needs to know. The below image shows the three modes of operation (tank draining, tank filling, & backwash).

<img src="/assets/scada/overview.gif" alt="">

#### 2. Security
Allows different operators to log into & out of the system.

<img src="/assets/scada/security.png" alt="">

#### 3. Controls
Allows for manual control of all devices, and general control of the system.

<img src="/assets/scada/Controls.png" alt="">

#### 4. Configuration
Allows for change of alarm and control system setpoints.

<img src="/assets/scada/Config.png" alt="">

#### 5. Runtime
This displays information such as system runtime and number of completed cycles.

<img src="/assets/scada/Runtime.png" alt="">

#### 6. Alarms
Shows alarm history, and allows operator to dismiss or reset alarms

<img src="/assets/scada/alarmsDemo-alarms.png" alt="">

#### 7. Trends
Graphically shows the trend history of pressure & tank levels, which are saved to a database.

<img src="/assets/scada/Trends.png" alt="">

#### 8. Data Aquisition

The course only had bear bones requirements for data logging, saving system information such as runtime, number of cycles, and historical pressure levels to an SQL database. I took this a step further and set the SQL server agent to periodically export the data tables to an excel file. 

Of course this only scratches the surface of what can be done with databases, and I am keen to further explore their uses in the automation industry.

### First Attempt

As mentioned, the course called for this project to be done on a number of platforms. I thought it would be neat to show the progess I made between implementations. This is the first attempt:

<img src="/assets/scada/eb_overview.png" alt="">

<img src="/assets/scada/eb_controls.png" alt="">

This was made using Weintek's Easybuilder 5000.

### User Experience

It's important to note that as of writing this I have minimal practical experience developing SCADA systems, so where I would usually go and ask an operator for advice or feedback, in these projects I was working in a vaccuum. 

I did some research on what makes a good and a bad HMI/SCADA system. Recurring themes were things like "only show the operator what they need to see", "don't clutter the screen". and "be careful using colours".

I went about following this advice, and typically found that I could trim a bit of fat from each screen. I also tried to make elements of the screen visually pop, to be quickly recognizable.  

I've seen a lot of HMI/SCADA's through internships & other work experience so far. Some look modern, a lot look like they're straight from the 80's, and there seems to be some argument as to which is superior. I put together a collage of HMI/SCADA's to draw inspiration from:

<img src="/assets/scada/collage.png" alt="">

And I found some useful colour themes & guides to base my SCADA screens on:

<img src="/assets/scada/colorScheme.png" alt="">
<p style="text-align: center;"> www.hmicons.com </p>

### Thoughts

The project was relatively simple overall, but I am proud of the result. Some things could be improved. I wonder if it would not be better to combine the controls & configuration screens in this project, but I can see it being necessary to seperate them in larger systems.
