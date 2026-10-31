function self_test()
rng(12);errors=randn(1,1000);gains=[1.8,.2,.015];dt=.01;memory=struct('integral',0,'previousError',0);old=0;older=0;incremental=0;
for error=errors
 [positional,memory]=pid_euler_step(error,memory,gains,dt);
 incremental=incremental+gains(1)*(error-old)+gains(2)*dt*old+gains(3)/dt*(error-2*old+older);
 assert(abs(positional-incremental)<1e-10);older=old;old=error;
end
fprintf('PASS: 1000 explicit-Euler positional/incremental PID comparisons.\n');
end
