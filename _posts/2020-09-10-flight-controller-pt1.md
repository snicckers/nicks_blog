---
usemathjax: true
layout: post
title: "Flight Controller: Part 1 - Single Axis Controller"
author: "Nick"
---

<p style="text-align: center;">  </p>

<iframe width="800" height="450" src="https://www.youtube.com/embed/UaFPJgMZJ5E" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

This is the first post in a series that covers the developement of a quadcopter flight controller. In this project I used an Arduino Uno, and MPU6050, and a number of other componenets.

My overall plan for the project was as follows:
1. Build a controller/stabiliser that worked on a single fixed axis
2. Model & Simulate a quadcopter in MATLAB/Simulink
3. Test the controller with unrestricted flight

Project files for the single axis controller can be found at the following link:
<p style="text-align: center;"> https://github.com/snicckers/single-axis-flight-controller </p>

### 1. Controller
<hr>

Modern vehicles, such as cars, jets, & helicopters, typically must be controlled using fly-by-wire systmes. This is because the number of controls or the reaction speeds required for stable operation exceeds human capabilities. This means that when an operator gives a command, the controller figures out how to execute that command safely. A simple, widely-used control method is the proportional-integral-derivative (PID) controller.

Each element in a PID controller acts to reduce error. In my case, the error is the difference between the measured angle and the desired angle. At this stage I was using a very simple inertial measurement unit (IMU) to detemrine the attitude, but later on I replaced it with my own implementation of a Magdwick Filter.

Deciding it would be best to incrementally develope the controller, I built a test stand and attached only one motor to the platform. Later I would move on to two and then four motors, but for now I needed to get comfortable with PID's.

<img src="/assets/images/single-axis-single-motor.jpg" alt="">
<p style="text-align: center;"> Single-axis single-motor stand </p>

As the name suggests, a PID controller has three componenets acting to reduce error. In our case, it modifies the output speed signal to the brushless motor.

$$ pid = K_p e(t) + K_i {\int} e(t) dt + K_d {d\over dt} e(t) $$

Each component has different behaviours that compliment each other. However, not all three elements must be used together: A PD controller can keep a quad mostly stable, and PI controllers are widely used in chemical & automation industries, where the error correction time-delay is not as critical.

**Proportional Control:** The actuator output is modified by a value proportional to the error. Big error means big motor output.  

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

/* Error: */
error = desired_angle - actual_angle;

/* Proportional Component: */
pid_p = k_p * error;

/* Derivative Component: */
// sample_time - the sime since previous error was recorded */
pid_d = k_d * ((error - previous_error) / sample_time);

/* Integral Component: */
// The range (in degrees) in which the integral acts
int k_i_thresh = 8; 

// Only use I-component within range of setpoint:
if (error < k_i_thresh && error > -k_i_thresh) 
  pid_i = pid_i * (k_i * error);    // Save pid_i as a global variable
if (error > k_i_thresh && error < -k_i_thresh)
  pid_i = 0;

/* Altogether: */
pid = pid_p + pid_i + pid_d;

// Calculate PWM time length:
pwm_out = throttle - pid;

{% endhighlight %}

Here is an example of this controller working with a single motor:

<p style="text-align: center;"> <iframe width="800" height="450" src="https://www.youtube.com/embed/RLtDDuf4XfQ" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe> </p>

The corresponding code change for two motors is very simple:

{% highlight c %}

// Calculate PWM width
pwm_right = throttle - pid;
pwm_left = throttle + pid;

// clamp PWM output
// esc's are programmed to stop working if they recieve a signal out of range
if (pwm_right < 1000) pwm_right = 1000;
if (pwm_right > 2000) pwm_right = 2000;
if (pwm_left < 1000) pwm_left = 1000;
if (pwm_left > 2000) pwm_left = 2000;

{% endhighlight %}

This is more compicated with four motors, as we have at least two degrees of freedom (roll & pitch) affecting each other. If you add a magnetometer to the IMU (to correct unstable yaw measurements) and a barometer (for reliable altitude measurement), then you have four degrees of movement that can be corrected by the controller: Roll, pitch, yaw, altitude. The control theory & implementation for this will be discuessed in a later post.

### 1.1 Derivative Kick
<hr>

After adding a radio reciever to the platform, I noticed that when the setpoint was rapidly changed (with a radio controller) there would be violent spike in pid values. This proved physically dangerous. A motor with it's thrust set at 30% (pwm of 1300 us) would shoot to 100% thrust uncontrollably if I changed the sepoint too quickly.

<img src="/assets/gifs/derivative-kick-example.gif" alt="">
<p style="text-align: center;"> PID output with time, notice the spikes </p>

I came to find that this is a common behaviour known as Derivative Kick. For more information:
- https://controlguru.com/pid-control-and-derivative-on-measurement/ 
- https://www.youtube.com/watch?v=KErYuh4VDtI 

As mentioned, the derivative component is the following:

$$ d = K_d {d\over dt} e(t) $$

Which in practice is:

$$ d = K_d {e(t) - e(t-1) \over d(t)} $$

So what if the change in error is very large? Ah. Output will be very large. The solution is to take the derivative of the process variable, rather than that of the error. In this case the process variable is our "pitch" angle variable. So the derivative term then becomes:

$$ d = - K_d { {\phi} (t) - {\phi} (t-1) \over d(t)} $$

In practice:

{% highlight c %}

pid_d = (-1.0f) * k_d * ((pitch - previous_pitch) / sample_time);

{% endhighlight %}

Note the change in sign. Here is a side-by-side comparison of the PID outputs. Red is using the error variable to calculate it's derivative action, blue is using the process variable.

<img src="/assets/images/derivativeKickExample.jpg" alt="">

So, using the process variable instead of the error variable for derivative control clearly removes these kicks. Problem sovled.

### 2. Tuning & Rapid Testing
<hr>

Tuning PID gains was a slow process. But tuning didn't need to be great to be effective with a single motor.

When moving to a two motor setup I tried some of the techniques I learned at university, such as the tedious Nichols-Ziegler method. The result worked but it was far too slow to be useful, so instead I used this method:

1. Set pid gains to zero
2. Increase the P gain until you have a stable oscillation
3. Increase the D gain until these oscillations cease (critically damped)
4. Repeat steps 2 & 3 until you're happy with the speed of the response
5. Increase the I gain until it reaches the setpoint in a minimul number of oscillations

Though lacking the engineering rigor that my degree has thus far conditioned me with, I stumbled upon some reassurance:

>**"**The very nice thing about tuning a PID controller is that you don’t need to have a good understanding of formal control theory to do a fairly good job of it.  Ninety percent of the closed-loop controller applications in the world do very well indeed with a controller that is only tuned fairly well...**"**

<p style="text-align: center;"> Tim Wescott, PID Without a PhD </p>

Using this method was tedious at first, having to reupload the program to make any changes. Seeking to tune the PID controller in real time, I acquired an unused TV-remote and an infrared sensor.

<img src="/assets/images/infrared.jpg" alt="">

I hooked them up to my Arduino and cut a 60 second tuning process down to a fraction of a second.

Resources:
- Instrumentation & Control Systems, 2nd Ed, William Bolton
- [link](https://www.wescottdesign.com/articles/pid/pidWithoutAPhd.pdf) - PID Without a PhD, Tim Wescott
- [link](https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops) - Robotics Stack Exchange, Pid Tuning

### 3. Reflection
<hr>

Being able to change the PID constants in real time and actually see the results as I changed them was a good education. The next post will focus on simulation & control of a quadcopter in MATLAB/Simulink, and finally unrestricted physical flight.
