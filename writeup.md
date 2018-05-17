## PID Control
---

**PID Control of a Simulated Vehicle**

The goals / steps of this project are the following:

* Implement a PID controller to provide steering controls for a simulated autonomous vehicle.  The vehicle must safely navigate itself around the track using only cross track error as input.

## [Rubric](https://review.udacity.com/#!/rubrics/513/view) Points
### 1. Compilation###

**Code should compile.**

- Done.  See compile instructions in the [README.md](README.md) file.

###2. Implementation###

**PID procedure follows what was taught in the lessons.**

- Done.  See [./src/PID.cpp](./src/PID.cpp).  For a given cross track error, $c_k$, at time $k$, the control input at each time step is computed as

  $$ u_k = -K_p\cdot c_k - K_d\cdot D_k - K_i\cdot E_k $$

  where

  $$D_k := c_k - c_{k-1}$$

  $$E_k:=\sum_{i=0}^k c_k$$

  and $K_p$, $K_d$, and $K_i$ are the _a priori_ control constants. 

### 3. Reflection ###

**Effect of each of the PID components**

- **Proportional Control (P)**:  Using just proportional control, we see that the steering behavior oscillates and quickly becomes unstable.

  See [./videos/p.mp4](./videos/p.mp4).

- **Derivative Control (D):**  Adding derivative control helps to stabilize the unstable behavior observed previously.  The car can now reasonably track the centerline of the road without wild oscillations.

  See [./videos/pd.mp4](./videos/pd.mp4).

- **Integrator Control (I):**  Adding an integrator control term helps to correct steering bias.  As the error accumulates over time, the control grows stronger.

  See [./videos/pid.mp4](./videos/pid.mp4).

**How were final parameters chosen**

- I tried several approaches to finding good control parameters.
- First, I did a little guess and check to see how each of the terms influenced steering behavior over short runs.  This helped to get a feel for their relative sizes, importance, and influence on steering behavior.
- Second, I also implemented the *Twiddle* approach as outlined in the lesson.  I built a `Twiddle` class that "watched" the simulation to keep track of total error, and adjusted the control parameters by coordinate descent.  
  Unfortunately, I did not obtain great success with this approach for the following three reasons.  
  1. Twiddle can get stuck in local minima.
  2. Twiddle optimized to find very small error control solutions for which the steering behavior was actually highly oscillatory with high frequency.  The constant steering slows the car down (since steering acts like braking on the car) so that cross track error can actually be made quite small, however, a human would never want to be a passenger in that car.  See [./videos/twiddle.mp4](./videos/twiddle.mp4).
  3. The Total Error metric for comparing performance of control parameters is very noisy.  Even after allowing long settling time (100 time steps) and long record time (500 time steps) the total error values still have a large variance.
- Finally, I used the insights from manual experimentation and human observation of the driving behavior to choose the parameters. 

### 4. Simulation###

**Successfully drive a lap around the track**

- **Steering Control**: Done.  See [./videos/pid.mp4](./videos/pid.mp4).

- **Throttle Control**: Also, I implemented a throttle controller.  The throttle controller tries to maintain a high speed along the straight portions of the track, and then slow down while cornering.  

  If the current steering angle is $\theta$, then the desired speed is

  $$\mathrm{v}_\text{desired} = 45 - 2|\theta|$$

  A recording of the racecar with both steering and throttle control can be seen in [./videos/throttle.mp4](./videos/throttle.mp4).

