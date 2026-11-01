function self_test()
rng(12);errors=randn(1,1000);gains=[1.8,.2,.015];dt=.01;
% Initialize as in the book: e(-1)=e(0), I(0)=0; compute u(0) first.
memory=struct('integral',0,'previousError',errors(1));
[incremental,memory]=pid_euler_step(errors(1),memory,gains,dt);
old=errors(1);older=old;
for error=errors(2:end)
 [positional,memory]=pid_euler_step(error,memory,gains,dt);
 incremental=incremental+gains(1)*(error-old)+gains(2)*dt*old+gains(3)/dt*(error-2*old+older);
 assert(abs(positional-incremental)<1e-10);older=old;old=error;
end
% Check the ROS PI convention with a nonzero discrete integral coefficient.
old=0;integral=0;command=0;kp=1.8;kid=.07;
for error=errors
 expected=kp*error+integral;
 command=command+kp*(error-old)+kid*old;
 assert(abs(command-expected)<1e-10);
 integral=integral+kid*error;old=error;
end
root=fileparts(mfilename('fullpath'));
source=fileread(fullfile(root,'ros_packages','pure_pursuit','src','pure_pursuit.cpp'));
assert(contains(source,'longitude_ki_ * longitude_perror_;'));
assert(~contains(source,'longitude_ki_ * relative_x;'));
other=fileread(fullfile(root,'..','Chapter11','CollabPlanner','pure_pursuit','src','pure_pursuit.cpp'));
assert(strcmp(source,other));
fprintf('PASS: 1000 PID samples, 1000 nonzero-gain PI samples, matching chapter-local ROS expressions.\n');
end
