# Chapter 10: Explicit-Euler PID Convention

`pid_euler_step` computes the command using the stored integral, then advances that integral by first-order explicit forward Euler. The equivalent incremental expression therefore uses the previous error in its integral increment. `self_test` compares both expressions over 1,000 input samples.

This small unsaturated algebraic example does not replace actuator limits, anti-windup, derivative filtering, or a physical tracking evaluation. The measured-error derivative uses a causal difference; it is not an alternative numerical state integrator. The Chapter11 supplied controller's zero integral gain makes this indexing correction neutral for its recorded default output.
