# Describe the effect each of the P, I, D components had in your implementation.

For the steering angle:

* The effect of the **P component** is the "reactivity" of the steering to the curve in the road. The greater the P component, the more sensitive/abrupt/ or reactive the steering will be. High values of P lead to overshooting the curve, and oscillations to correct for the overshooting.
* The effect of the **I component** is that it can correct for longer-term accumulated bias in the steering.
* The effect of the **D component** is that it dampens the steering angle somewhat and can correct any future overshooting behavior from a high P component before it is too late. A D component results in dampening the oscillations from a high P component.

For the throttle, I implemented a PD controller in such a way that the speed is inversely proportional to the cross-track error. In other words, if the car is far away from the desired position (e.g., in sharp curves), the car will slow down. Analogously, if the car is close to the desired position, the car will speed down. The P component of speed represents, again, the reactivity. The D component dampens the reactivity and makes the changes in throttle less abrupt.

#Describe how the final hyperparameters were chosen.

* I started out with a manual tuning of the PID coefficients for the steering angle, with a constant throttle of 0.3. I started with just a P controller and increased the value of Kp until I observed oscillations in the steering angle. Subsequently, I increased the value of Kd until the oscillations were dampened. Finally, I included a small value of Ki (0.0001). This gave me an MSE of around 0.35.
* Next, I ran a twiddle optimization for the steering angle with the manually tuned PID coefficients from the previous step as initial guesses. The throttle was still held constant to 0.3. I started out with step sizes that were about 1/10th of the manual guesses.  This reduced the MSE to around 0.15
* Next, I switched on the PD controller for the throttle. I manually tuned the values for Kp and Kd for the throttle. I first increased the Kp coefficient until the speed increased and decreased appropriately in the path (somewhere between 30 and 60 mph), I then increased the Kd coefficient to reduce the throttle oscillations and get a smoother change in speed and no excessive breaking. This step yielded in much higher speeds, but the MSE increased again somewhat to 0.25.
* Finally, I ran a twiddle optimization again, but now with the throttle PD controller on. I started off the twiddle optimization with the optimal PID coefficients I got with a constant throttle. This reduced to MSE to about 0.20.