# PID control Project
Self-Driving Car Engineer Nanodegree Program

This project works with Udacity self driving simulator (for term 2) and
uses a PID control in order to steer a vehicle around a track.

## PID controller
The PID controller a controller designed to make some a controlled paramter follow a certian value or series of values.
This controller is used to regulate the parameter over time which means, the controlled parameter will follow the its designated 
value more accuratly over time.

The controller is using an error from the target value in order to correct the controlled parameter.
The controller is composed of three parts:
* Proportonal - Used to correct the controlled value by moving proportianly against the error.
* Integral  - Used to provide increasing force in order to help reducing the error to zero. Used when the parameter has bias.
* Derivative - Helps the controller arrive smoothly to the minimal error value and preventing ocillations around the min value.

## Expected values in project
I expected to have a proportonal part that is relativly small to the derivative part.
The reason for that is that the derivative part need to be strong enough to smooth the controlled parameter thwards the minimal error value and not let the proportonal part increase the error again and cause ocillations.
I expected the integral part to have some value in order to correct the simulator bias if it has one.

In order to reach the desired result I've implemented twiddle in order to tweak the controller parameters.
After 6 hours that the twiddle algorithem ran, the result were `[1.7341, 0, 6.78751]` with MSE of 0.23 on 1000 samples (twiddle tolarence was 0.01).

As expected the D coeff was larger then the P coeff. But for my surprise the I coeff was zero which means there was no bias in simulator.


