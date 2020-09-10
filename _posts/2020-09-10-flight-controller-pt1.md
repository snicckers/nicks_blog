---
usemathjax: true
layout: post
title: "Flight Controller: Part 1 - Single Axis Controller"
author: "Nick"
---

<p style="text-align: center;">  </p>

<p style="text-align: center;"> <iframe width="800" height="450" src="https://www.youtube.com/embed/v6ljsSUh884" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe> </p>

This is the first post in a series that covers the developement of a quadcopter flight controller. In this project I used an Arduino Uno, and MPU6050, and a number of other componenets.

My overall plan for the project was as follows:
1. Build a controller/stabiliser that worked on a single fixed axis
2. Build a controller that worked in 3 degrees of freedom, attatched to a fixture
3. Test the controller with unrestricted flight

Project files for the single axis controller can be found at the following link:
<p style="text-align: center;"> https://github.com/snicckers/single-axis-flight-controller </p>

## Section 1 - Controller
<hr>

The majority of modern craft, from power steering in cars, to quadcopters, to jet aircraft, must be piloted using a fly-by-wire system. Typically, reaction speeds and number of controls required for stable operation exceed human capabilities. The result is that when the pilot gives a command, the flight controller figures out how to execute that command. A simple, widely-used control method is the proportional-integral-derivative (PID) controller, which acts to reduce error.

In my case, the error is the difference between the measured angle and the desired angle. At this stage I was using a very simple inertial measurement unit (IMU) to detemrine the attitude, but later on I replaced it with my own implementation of a Magdwick Filter, discussed in part 2. 

Deciding it would be best to incrementally develope the controller, I built a test stand and attached only one motor to the platform. Later I would move on to two and then four motors, but for now I needed to get comfortable with PID's.

<img src="/assets/images/single-axis-single-motor.jpg" alt="">
<p style="text-align: center;"> Single-axis single-motor stand </p>

As the name suggests, a PID controller has three componenets acting to reduce error. In our case, it modifies the output speed signal to the brushless motor.

$$ pid = K_p e(t) + K_i {\int} e(t) dt + K_d {d\over dt} e(t) $$

Each component has different behaviours that compliment each other. However, not all three elements must be used together: A PD controller can keep a quad mostly stable, and PI controllers are widely used in chemical & automation industries, where the error correction time-delay is not as critical.

**Proportional Control:** The actuator output is modified by a value proportional to the error. Big error means big motor output. The gain $$ K_p $$ determines the power of the P-component. 

$$ p = K_p e(t) $$

When using only proportional control, the result tends to be an oscillation around the setpoint.

**Derivative Control:** The actuator output is modified in such a way that it will attempt to resist a change in error. It will not try to correct the error, only opposing change in error. In our case, it will resist any change in rotation.

$$ d = K_d {d\over dt} e(t) $$


**Integral Control:** This is the summation of the error over a desired time period. Alone it acts like a delayd P-component, leading to a similar oscillation around the setpoint. But, when acting together with a P-component, the I-component tends to dampen oscillaiton. 

$$ i = K_i {\int} e(t) dt $$

The I-component can dangerously overshoot the setpoint if too much history is recorded, a behaviour known as integral wind-up. To prevent this, one can enact the I-component only when the error is close to the setpoint.

**Implementation:**

The influence of these three components is determined by the corresponding gains, $$ K_p $$, $$ K_i $$, and $$ K_d $$.

{% highlight c %}

// Error:
error = desired_angle - actual_angle;

// Proporitonal Component:
pid_p = k_p * error;

// Derivative Component:
// sample_time - the sime since previous error was recorded */
pid_d = k_d * ((error - previous_error) / sample_time);

// Integral Component:
// The range (in degrees) in which the integral acts
int k_i_thresh = 8; 

/* Only carry out integral within range: */
if (error < k_i_thresh && error > -k_i_thresh) 
  pid_i = pid_i * (k_i * error);    // Save pid_i as a global variable
if (error > k_i_thresh && error < -k_i_thresh)
  pid_i = 0;

// Altogether:
pid = pid_p + pid_i + pid_d;

// Calculate PWM width:
pwm_out = throttle - pid;

/* You will also want to cap the motor PWM outputs upper and lower bounds */

{% endhighlight %}

It is pretty simple to implement, however there are whole books written on PID control - not all of PID controller use is as trivial as this.

Here is an example of this controller working with a single motor:

<p style="text-align: center;"> <iframe width="800" height="450" src="https://www.youtube.com/embed/RLtDDuf4XfQ" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe> </p>

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

Part 2 is next up, focusing on the implementation of a Madgwick filter, and a Mahony filter. 