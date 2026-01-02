The Physics & Math: Leverage your Physics degrees 
    - Explain the PID controller code you wrote.
    - Reference how you defined the PIDController class to handle error history and integral/derivative calculations 
    - Explain your safety cutoffs (MAX_TILT_DEGREES = 45.0) 
This shows you think about safety and system limits, which is relevant to your NDT and instrumentation background 

The "Drift" Issue (Critical Analysis):
    You mentioned the robot drifts because it doesn't use encoders yet. Documenting this is actually a strength.
    Explain why it happens: "The current control loop relies solely on tilt angle from the BNO085 
        Without wheel encoder feedback, the robot cannot detect linear velocity or position, leading to drift. Future iterations would fuse encoder data to establish a position hold."
This proves you understand the system theoretically, even if you haven't implemented the code yet.


This is where you flex your Physics and Instrumentation background.
    - The Math: Explain the PID loop.
    - Code Snippets: Highlight your PIDController class. Explain how you calculate the integral term (self.integral += error * dt) and why you implemented the safety cutoff (MAX_TILT_DEGREES = 45.0) 
Why this matters: It proves you understand signal processing and error analysis, not just copying code.

(The "Drift" Explanation)
This is the most important file for your credibility. You admitted the robot drifts because it lacks encoder feedback. Turn this weakness into a strength.
    The Problem: "The robot balances vertically but drifts horizontally."
    The Root Cause: Explain that the BNO085 provides orientation (tilt) but not position or velocity 
Without wheel encoders, the PID loop corrects the angle but ignores the linear displacement.
The Solution (Future Work): "Implement sensor fusion using rotary encoders to add a velocity control loop on top of the angle control loop."
Why this helps: It shows you know how to fix it, even if you haven't done it yet.

C. docs/hardware.md
    Since you don't have perfect notes, just take high-quality photos of the current wiring and the 3D printed chassis.