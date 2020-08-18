---
usemathjax: true
layout: post
title: "Flight Controller: Part 1 - Single Axis Controller"
author: "Nick"
---

<p style="text-align: center;">  </p>

This blog series documents the development of a quadcopter flight controller using an Arduino Uno. It is written for my own purposes, and I would not recommend using it as an educational resource. This part of the series focuses on the creation of a single axis stabiliser, with one and then two motors, fixed to a test platform. Here are some of the resources I used during the early stages of this project:

Upon initial research I discovered the tutorial series by Joop Brokking. However I did not want to wholesale copy/paste his or others' code, rather I wanted to understand what I was building step by step, mostly for my own satisfaction rather than any practical purpose. I did use his & others's projects as references while working on this first part of the project.

My overall plan for the project was as follows:
1. Build a controller that worked on a single fixed axis
2. Build a controller that worked in 3 degrees of freedom, fixed to a test stand
3. Test the controller with unrestricted flight

The focus of this post is step 1. Steps 2 & 3 will be covered in the third post. The second post will focus on instrumentation.

## Section 1 - First Steps
<hr>
I started off with simple Inertial Measurement Unit (IMU), based on sample code I found on some website - If you do a google search you're guaranteed to quickly find something similar. It looks like this:

{% highlight c %}
void calculate_attitude(int sensor_data[]){
  /* Attitude from accelerometer */
  a_magnitude = sqrt(pow(a_read_ave[0], 2) + pow(a_read_ave[1], 2) + pow(a_read_ave[2], 2));
  a_roll = asin((float)a_read_ave[1] / a_magnitude) * rad_to_degrees; // X
  a_pitch = asin((float)a_read_ave[0] / a_magnitude) * rad_to_degrees * (-1); // Y

  /* Attitude from Gyroscope */
  float t = (micros() - elapsed_time);
  elapsed_time = micros();
  double lsb_coefficient = (1 / 32.8) * ((t) / 1000000);
  g_roll += sensor_data[4] * lsb_coefficient * 1.09;
  g_pitch += sensor_data[5] * lsb_coefficient * 1.09;
  //1.09 = coefficient that makes gyro attitue match accel attitude.

  /* Complimentary FIlter */
  float hpf = 0.9996; // High Pass Filter
  float lpf = 1.0 - hpf; // Low Pass Fbilter
  g_roll = hpf * g_roll + lpf * a_roll;
  g_pitch = hpf * g_pitch + lpf * a_pitch;
}

{% endhighlight %}

The instrumentation gets a lot more complicated later on, but this will suit just fine for working with a single axis. Here Euler angles found by integrating gyro rates from an MPU6050. MEMS gyro rates always gradually drift, causing inaccuracies over long time periods. The solution is to correct the drift using accelerometer data using a complimentary filter.  This is a huge oversimplification, and it will be addressed further in the part 2. There is no rotation matrix for the angles coming from the gyro, so no tilt compensation is performed in this IMU (aside from the accelerometer slowly correcting the angle). This is not the final IMU that I went with, but for getting a controller to work on a single axis it does the job.

I built an admittedly terrible setup, and programmed a simple Proportiona-Integral-Derivative Controller. It performed awfully, & in retrospect I should have realized this ahead of time:

<img src="/assets/gifs/single-axis-first-try.gif" alt="">
<p style="text-align: center;"> lol </p>

After this initial foray, now early in the second year of my degree, I lost interest in the project. I did not return to it for over a year.

## Section 2 - Controller
<hr>

Fast forward a year, with a class in control systems under my belt, I decided to revisit the project. I built a new, much safer, single axis test stand. I started out by attaching only 1 motor to the platform, thinking it would be easier to learn the implementation & tuning of PID controllers.

<img src="/assets/images/single-axis-single-motor.jpg" alt="">
<p style="text-align: center;"> Single-axis single-motor stand </p>

Due to the instability of a quadcopter platform, it has to be fly-by-wire. So when the pilot gives it a command, and the flight controller figures out how to execute that command in a stable fashion. When hovering in a stationary position, the controller still has to do a lot of work to keep the craft in a stable position (imagine a gust of wind hitting the quadcopter, or if, due to inaccuracies due to cheap manufacturing practices, one motor is more powerful than another).

There are some very fancy ways of doing this, but an old tried and tested method is the Proportional, Integral, & Derivative controller. Like the name suggests, the PID controller has three components that act to eliminate error. Say you have an error between the orientation that the quadcopter is on, and the desired orientation (with no user input, the desired orientation would be parallel to the ground). 

$$ e(t) = setpoint - {\phi} $$

Or:

{% highlight c %}

error = desired_angle - pitch;

{% endhighlight %}

Then the controller will attempt to correct that error by modifying the output to the actuator that influences the orientation - in this case brushless motor speed. It does this by modifying the output by the following.

$$ pid = K_p e(t) + K_i {\int} e(t) dt + K_d {d\over dt} e(t) $$

But what does this mean?

**Proportional Control:** The actuator output is modified by a value proportional to the error. A big error will have a correspondingly large modification to the output. This is tuned / controlled with a constant $$ K_p $$. When using only the proportional component, you will tend to get an oscillating motion around the setpoint. 

$$ p = K_p e(t) $$

Here's the implementation:

{% highlight c %}

pid_p = k_p * error;

{% endhighlight %}

**Derivative Control:** A value based on how fast the error is changing. The value will only get large when there's a large change in error over a short period. This tends to oppose any rapid change in motion. So if the proportional component very quickly changes the motor speed to correct a large error, then the derivative component will attempt to slow down the motor as the error changes, effectively opposing the proportional component. Again, this is tuned by a constant $$ K_d $$. 

$$ d = K_d {d\over dt} e(t) $$

Here's the implementation:

{% highlight c %}

/* sample_time - the sime since previous error was recorded */
pid_d = k_d * ((error - previous_error) / sample_time);

{% endhighlight %}

**Integral Control:** Finally, the integral component is the summation of the error over some desired time period. This can be dangerous if it's allowed too much power, and the one I had the most trouble with. One method to prevent the output change from becoming very large over a long time period, the integral component is only allowed to record the error's history when it is close to the setpoint. It is useful for smoothing out the output when close to the setpoint. In the case of a quadcopter, when tuned correctly, it can provide a very stable platform for photographers or other media professionals to mount camera equipment.

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

Note that the PWM range for the brushless motor is 1000 to 2000 microseconds, so you'll want to put a clamp on them so that the brushless motor doesn't receive an order below 1000 or above 2000:

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

Scroll down to section 4 to see it working with two motors. The code change for two motors is very simple:

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

As the project moved along I started using a potentiometer, and later a radio transmitter / controller, to change the setpoint, as a preparation for radio control in the future. I soon noticed that if I rapidly changed the setpoint, there would be an enormous spike in the controller's output values to the motor. For example, setting the throttle to 1300 with a reasonably well tuned controller, if I ordered a near-instant change in the setpoint, the output value would shoot to the maximum (2000) instantly. 

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

<img src="/assets/gifs/derivative-kick-corrected-example.gif" alt="">
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

Yes this method is sort of winging it, but while doing some reading into the topic I came upon this:

>**"**The very nice thing about tuning a PID controller is that you don’t need to have a good understanding of formal control theory to do a fairly good job of it.  Ninety percent of the closed-loop controller applications in the world do very well indeed with a controller that is only tuned fairly well...**"**

<p style="text-align: center;"> Tim Wescott, PID Without a PhD </p>

So with that ressuring message I started off with my quest to tune a fast-acting controller. However, changing the values, waiting for the program to upload, observing the results, and repeating, was a very time consuming and tedious activity.

The solution? Change the values in real time! So I acquired an "unused" tv-remote, and I was lucky enough to have an infrared sensor hanging around. 

<img src="/assets/images/infrared.jpg" alt="">
<background-image: url("~/assets/foobar.jpg");>

Now I could instantly change my PID gains without having to re-upload the entire program, cutting a 60 second process down to a fraciton of a second, and it injected a large dose of fun into the tuning process. 

I used the following resources in this section:
- Instrumentation & Control Systems, William Bolton
- [link](https://www.wescottdesign.com/articles/pid/pidWithoutAPhd.pdf) - PID Without a PhD, Tim Wescott
- [link](https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops) - Robotics Stack Exchange, Pid Tuning

## Section 4 - Demonstration & Reflection
<hr>

With all that said I managed to cobble together a reasonably well tuned flight stabilizer working on a single axis with two motors. Being able to change the PID constants in real time and actually see the results as I changed them was a very helpful education. I feel that I have walked away with a deeper understanding of this type of controller, than if I had just used the equations to tune it, or used something like Matlab or TCLab.

Here is a video demonstration of the final single-axis controller:

<p style="text-align: center;"> <iframe width="800" height="450" src="https://www.youtube.com/embed/v6ljsSUh884" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe> </p>

Why did I do this? I'm not sure, I thought it would be fun. Note that this project was carried out over the course of the last few years, and the project stood abandoned for long periods of time. When I began, I was at the start of an engineering degree, and didn't really know anything about anything, although to be honest I feel that I know less now that I did then. At the beginning I knew very little about programming or electronics, now I would say I am quite comfortable (for an undergraduate Mechanical Engineer) with procedural & object-oriented programming. Going forward I'd like to balance out my skillset with some electrical knowledge.

Part 2 is next up, focusing on the implementation of a Madgwick filter, and a Mahony filter. 