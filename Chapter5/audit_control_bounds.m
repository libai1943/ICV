function report=audit_control_bounds(state,control,horizon,task,forwardOnly)
% Exact extrema of v, steering, and acceleration under piecewise-constant controls.
steps=size(control,2);dt=horizon/steps;minimum=inf;maximum=-inf;steer=0;accel=0;terminal=0;
for car=1:size(state,3)
 v=state(4,1,car);phi=state(5,1,car);a=state(6,1,car);
 for k=1:steps
  j=control(1,k,car);omega=control(2,k,car);times=[0,dt];if abs(j)>1e-12,t=-a/j;if t>0&&t<dt,times(end+1)=t;end,end
  speeds=v+a*times+.5*j*times.^2;minimum=min(minimum,min(speeds));maximum=max(maximum,max(speeds));steer=max([steer,abs(phi),abs(phi+omega*dt)]);accel=max([accel,abs(a),abs(a+j*dt)]);
  v=v+a*dt+.5*j*dt^2;phi=phi+omega*dt;a=a+j*dt;
 end
 terminal=max(terminal,max(abs([v;phi;a]-state(4:6,end,car))));
end
lower=-task.maxSpeed;if forwardOnly,lower=0;end
violation=max([0,lower-minimum,maximum-task.maxSpeed,steer-task.maxSteering,accel-task.maxAcceleration]);
report=struct('minimum_velocity',minimum,'maximum_velocity',maximum,'maximum_steering',steer,'maximum_acceleration',accel,'maximum_continuous_bound_violation',violation,'forward_terminal_dynamic_drift',terminal,'passed',violation<1e-3&&terminal<1e-3);
end
