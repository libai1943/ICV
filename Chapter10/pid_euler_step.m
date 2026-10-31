function [command,memory]=pid_euler_step(error,memory,gains,dt)
% Unsaturated PID: output from stored integral, then explicit Euler update.
assert(dt>0);command=gains(1)*error+gains(2)*memory.integral+gains(3)*(error-memory.previousError)/dt;
memory.integral=memory.integral+dt*error;memory.previousError=error;
end
