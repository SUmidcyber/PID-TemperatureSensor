# PID-TemperatureSensor

# What is the advantage over normal algorithms?

## This code is a real application, I know it has some shortcomings, but now according to the normal algorithm
If we say what is the advantage, This code is used to simulate a basic PID control algorithm.used and has some shortcomings in real applications. In short, we know.
For Simulation Purposes: This code uses the PID control algorithm without a temperature sensor or heater.
used to simulate. Because it is not connected to a real physical system, real-time
It is not suitable for use in applications. However, to understand the basic working principle of the algorithm and It is useful for testing parameters.
No Actual Data Entry: In the TemperatureSensor class, a fixed temperature value (25) is returned, It's different in the real world, but here you can set the temperature as you wish.
No Real Processor and Time Mechanism: control loop using time.sleep(1) i.e. delay I added
Parameters Fixed: In this code, PID control parameters (Kp, Ki, Kd) are specified as fixed, which is our
in terms of understanding how to make our job easier

<a href="https://imgur.com/X2EvfF5"><img src="https://i.imgur.com/X2EvfF5.png" title="source: imgur.com" /></a>
