# PIDControlloer
PID controlloer Udactiy project. This project is designed and implemented for driving a car in Udactiy simulator with controlling steering angle and throttle, given CTE.

The goal is to successfully drive the car in simulator by using Proportional, Differential, and integral terms in calculating the driving angle so as the car
drives in the given trajectory. In addition to implementing such a steering angle calculation, this project also provides learning of these
parameters along with the throttle control.

To run the project, you will need the Udacity simulator.

Steps to run:
mkdir build
cd build
cmake ..
make
./pid

## PID effect
### P (Propotional component)

This component controls the steering angle of a car such that car directly drifts towards the given trajectory.
Larger the P, faster the car tries to get to the given trajectory. For the same reason, in given simulator, I found the car quickly starts overshooting and
undershooting the trajectory with the increase in this value. This effect was quite evident with the increase in speed. In other words, descreasing this parameter
helped when I found the car oscillating a lot. On the other hand, the car couldn't recover from the unexpected turns with the very small value of this parameter.
I also found this parameter making the car more sensitive to drifts from trajectory.

### I (Integral component)

This compoenent controls the angle calculation based on the total error the car has encountered so far. Here,
the error indicates distance from the given trajectory. This component is especially helpful when the car steering components have some bias i.e.
turning the steering to certain degree reflects more (or less) angle change of the car than expected. Measuring the erros over a period of time
can detect such a bias. Hence, this term is used to correct such biases. I believe the given car simulator did not have such a bias. The car
was designed to its perfection and no mechanic servicing this car messed things up :-). Hence, it did not have a lot of effect on the accuracy.
In fact, lowering the value of this parameter helped the car drive better.

### D (Differential component)

This compoenent controls the steering angle such that the car does not overshoot to the trajectory. It operates on
the previous error and the current error. If we find that the car is getting closer to the trajectory then we should reduce the turning angle
so that the car doesn't overshoot or undershoot the trajectory. In other words, this component mitigates the effect of the previous component.
I fould a relativey large value of this compoenent helpigng the car to remain on its trajectory with minimum oscillations. A larger value of
this compoenent made the car drive smoother. On the other hand, a larger value of this makes it difficult for the car to recover from unexpected
drifts.

In addition to these parameters, I also found the importance of the throttle value in driving this car. The current version includes
a static change in throttle based on the steering angle. However, more sophisticated control of the steering angle could certainly benefit
the car drive smoother and with the higher speed.

### Choice of the co-efficients

It is extremely important to correctly choose the co-efficents that control P, I, and D for the reasons mentioned above.
I started with co-ordinate accent to learn these paramters. However, a poor starting choice of these parameters results the car
to quickly get out of the road stopping the simulation. Hence, the initial values of these co-efficients were chosen with trial and error
experiments with the intitutions listed above. After finding values that atleast kept the car on the road, I ran co-ordinate accent to
fine tune the co-efficients.
