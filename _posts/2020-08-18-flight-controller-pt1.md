---
usemathjax: true
layout: post
title: "Flight Controller: Part 1 - Single Axis Controller"
author: "Nick"
---

<p style="text-align: center;">  </p>

This is the first in a series of posts that document the developement of a quadcopter flight controller, using an Arduino Uno, an MPU6050, and a number of other components.

My overall plan for the project was as follows:
1. Build a controller that worked on a single fixed axis
2. Build a controller that worked in 3 degrees of freedom, fixed to a test stand
3. Test the controller with unrestricted flight

This first part of the series focuses on the creation of a single axis flight stabilizer. You can find the project files discussed below at the following link:
<p style="text-align: center;"> https://github.com/snicckers/single-axis-flight-controller </p>

## Section 1 - Controller
<hr>

After initially giving up on the project a year earlier, I revisited the project with a class in control systems under my belt. I built a test stand and started out by attaching only 1 motor to the platform, thinking it would be easier to learn the implementation & tuning of PID controllers on one axis. The inertial measurement unit (IMU) used while developing this controller was a very simple complimentary filter between accelerometer & gyroscope instruments, but the final IMU used is a Madwick Filter that I implemented, which will be discussed in part 2. For working with a single axis, either IMU will do.

<img src="/assets/images/single-axis-single-motor.jpg" alt="">
<p style="text-align: center;"> Single-axis single-motor stand </p>

The majority of modern craft, from power steering in cars, to quadcopters, to jet aircraft, must be piloted using fly-by-wire. This is typically due to the number of controls and the reaction speeds required being far too high for a human to handle. What this means is that when the pilot gives a command, the flight controller figures out how to execute that command. Even when hovering in a stationary position, the controller must work to keep the craft in stable condition (for example, a gust of wind may hit the craft, or due to manufacturing intolerances one motor may be more powerful than another).

A tested method for achieving this is the Proportional, Integral, & Derivative controller. As the name suggests, the PID controller has three components that act to reduce an error. 

Say you have an error - the difference between the orientation that the quadcopter is on, and the desired orientation (with no user input, the desired orientation would be parallel to the ground). 

$$ e(t) = setpoint - {\phi} $$

Or:

{% highlight c %}
error = desired_angle - pitch;
{% endhighlight %}

Then the controller will attempt to correct that error by modifying the output to the actuator that influences the orientation - in this case, brushless motor speed. It does this by modifying the output by the following.

$$ pid = K_p e(t) + K_i {\int} e(t) dt + K_d {d\over dt} e(t) $$

What does this mean?

**Proportional Control:** The actuator output is modified by a value proportional to the error. A big error will have a correspondingly big modification to the output. This is tuned / controlled with a constant $$ K_p $$ gain. When using only the proportional component, you will tend to get an oscillating motion around the setpoint. 

$$ p = K_p e(t) $$

Implementation:

{% highlight c %}

pid_p = k_p * error;

{% endhighlight %}

**Derivative Control:** A value based on how fast the error is changing. The value will only get large when there's a large change in error over a short period. This tends to oppose any rapid change in motion. On its own, the derivative component will only act to oppose a change in the setpoint, but it will not correct error. This is also tuned by a constant $$ K_d $$ gain. When combined with proportional action, a PD controller can do most of the work required to rapidly reach the setpoint. 

$$ d = K_d {d\over dt} e(t) $$

Implementation:

{% highlight c %}
/* sample_time - the sime since previous error was recorded */
pid_d = k_d * ((error - previous_error) / sample_time);
{% endhighlight %}

**Integral Control:** Finally, the integral component is the summation of the error over some desired time period. Wh`en working alone it acts like a delayed proportional controller, but adding proportional control tends to dampen to oscillation. It suffers from integral wind-up, which is when too much error history is accumulated. This can cause a dramatic overshoot of the setpoint, so integral action is often prevented from acting unless it is within a short range of the setpoint. It has its own gain, $$ K_i $$.

$$ i = K_i {\int} e(t) dt $$

Implementation:

{% highlight c %}

/* The range (in degrees) in which the integral acts: */
int k_i_thresh = 8; 

/* Only carry out integral within range: */
if (error < k_i_thresh && error > -k_i_thresh) 
  pid_i = pid_i * (k_i * error);
if (error > k_i_thresh && error < -k_i_thresh)
  pid_i = 0;

{% endhighlight %}

You will have to save "pid_i" as a global variable to preserve the history between scans. You then sum the components & add or subtract them from the throttle like so:

{% highlight c %}

pid = pid_p + pid_i + pid_d;

/* Calculate PWM width. */
pwm_out = throttle - pid;

{% endhighlight %}

- insert gif of integral & integral + pid action

Note that the PWM range for the brushless motor is 1000 to 2000 microseconds, so you'll want to put a clamp on them such that the brushless motor doesn't receive an order below 1000 or above 2000:

{% highlight c %}

/* Clamp the maximum & minimum pid values*/
if (pid < -1000) pid = -1000;
if (pid > 1000) pid = 1000;

/* clamp PWM values (pwm_out is the output to the motor) */
if (pwm_out < 1000) pwm_out = 1000;
if (pwm_out > 2000) pwm_out = 2000; 

{% endhighlight %}

It is pretty simple to implement, however there are whole books written on PID control - not all of PID controller use is as trivial as this.

Here is an example of this controller working with a single motor:

<p style="text-align: center;"> <iframe width="800" height="450" src="https://www.youtube.com/embed/RLtDDuf4XfQ" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe> </p>

Scroll down to section 4 to see it working with two motors. Below is a diagram for this control system using two motors:

<img src="/assets/images/single-axis-flight-stabiliser-diagram.png" alt="">

The corresponding code change for two motors is very simple:

{% highlight c %}

/* Calculate PWM width. */
pwm_right = throttle - pid;
pwm_left = throttle + pid;

/* clamp PWM values. */
//----------Right---------//
if (pwm_right < 1000) pwm_right = 1000;
if (pwm_right > 2000) pwm_right = 2000;

//----------Left---------//
if (pwm_left < 1000) pwm_left = 1000;
if (pwm_left > 2000) pwm_left = 2000;

{% endhighlight %}

## Section 2.1 - Derivative Kick
<hr>

As the project moved along I started using a radio reciever / controller to change the setpoint, as a preparation for radio control in the future. I soon noticed that if I rapidly changed the setpoint, there would be an enormous spike in the controller's output values to the motor. For example, setting the throttle to 1300 with a reasonably well tuned controller, if I ordered a near-instant change in the setpoint, the output value would shoot to the maximum (2000) instantly. 

<img src="/assets/gifs/derivative-kick-example.gif" alt="">
<p style="text-align: center;"> Notice the spikes </p>

I did some research and quickly found out this is a behaviour called "Derivative Kick".
- https://controlguru.com/pid-control-and-derivative-on-measurement/ 
- https://www.youtube.com/watch?v=KErYuh4VDtI 

As mentioned earlier, the derivative component is the following:

$$ d = K_d {d\over dt} e(t) $$

Which in practice is:

$$ d = K_d {e(t) - e(t-1) \over d(t)} $$

So what if the change in error is very large? Ah. Then the output will be very large. The solution to this is to take the derivative of the process variable, rather than that of the error. In this case the process variable is our "pitch" angle variable. So the derivative term then becomes:

$$ d = - K_d { {\phi} (t) - {\phi} (t-1) \over d(t)} $$

In practice:

{% highlight c %}

pid_d = (-1.0f) * k_d * ((pitch - previous_pitch) / sample_time);

{% endhighlight %}

Note the change in sign. Here is a side-by-side comparison of the PID outputs. Red is using the error variable to calculate its derivative action, blue is using the process variable.

<img src="/assets/images/derivativeKickExample.jpg" alt="">
<p style="text-align: center;"> Notice blue is almost identical to red, only without the spikes </p>

You can see that when using the process variable is almost the same as when using the error - only there are no spikes when the setpoint rapidly changes. 

## Section 3 - Tuning & Rapid Testing
<hr>

When using only a single motor, I tuned the PID gains by eye. it was a slow process, but it didn't need to be great to work.

When moving to a two-motor setup the first thing I did was try out the Nichols-Ziegler PID tuning method that I remembered from a uni course. It was a pain to figure out the oscillation period, but eventually I had a response that reached the setpoint - only it did so very slowely. From here I had a few options, I could do it with Matlab or a Python PID library, but I fould this method of doing it by eye:

1. Set pid gains to zero
2. Increase the P gain until you have a stable oscillation
3. Increase the D gain until these oscillations cease (critically damped)
4. Repeat steps 2 & 3 until you're happy with the speed of the response
5. Increase the I gain until it reaches the setpoint in a minimul number of oscillations

I felt that this method was sort of "winging it", it lacked the engineering rigor that my degree had conditioned me with. But when reading into PID control I found the following:

>**"**The very nice thing about tuning a PID controller is that you don’t need to have a good understanding of formal control theory to do a fairly good job of it.  Ninety percent of the closed-loop controller applications in the world do very well indeed with a controller that is only tuned fairly well...**"**

<p style="text-align: center;"> Tim Wescott, PID Without a PhD </p>

With this reassurance I started off with my tuning quest. However, changing the values, waiting for the program to upload, observing the results, and repeating, was a very time consuming and tedious activity.

The solution? Change the values in real time! So I acquired an "unused" tv-remote, and I was lucky enough to have an infrared sensor hanging around. 

<img src="/assets/images/infrared.jpg" alt="">

After hooking them up to my Arduino, PID gains could be changed in real time without having to re-upload the entire program, cutting a 60 second process down to a fraciton of a second. It also injected a dose of fun.

I used the following resources in this section:
- Instrumentation & Control Systems, 2nd Ed, William Bolton
- [link](https://www.wescottdesign.com/articles/pid/pidWithoutAPhd.pdf) - PID Without a PhD, Tim Wescott
- [link](https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops) - Robotics Stack Exchange, Pid Tuning

## Section 4 - Demonstration & Reflection
<hr>

With all that said I managed to cobble together a reasonably well tuned flight stabilizer working on a single axis with two motors. Being able to change the PID constants in real time and actually see the results as I changed them was a very helpful education. I feel that I have walked away with a deeper understanding of this type of controller, than if I had just used the equations to tune it, or used something like Matlab or TCLab.

Here is a video demonstration of the final single-axis controller:

<p style="text-align: center;"> <iframe width="800" height="450" src="https://www.youtube.com/embed/v6ljsSUh884" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe> </p>

Part 2 is next up, focusing on the implementation of a Madgwick filter, and a Mahony filter. 