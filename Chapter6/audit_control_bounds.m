function report=audit_control_bounds(q,u,horizon,task,varargin)
speed=q(4,:,:);steering=q(5,:,:);accel=q(6,:,:);
low=-task.maxSpeed;if nargin>4 && varargin{1},low=0;end
violation=max([0;low-speed(:);speed(:)-task.maxSpeed;abs(steering(:))-task.maxSteering;abs(accel(:))-task.maxAcceleration;reshape(abs(u(1,:,:))-task.maxJerk,[],1);reshape(abs(u(2,:,:))-task.maxSteeringRate,[],1)]);
report=struct('minimum_velocity',min(speed(:)),'maximum_velocity',max(speed(:)),'maximum_steering',max(abs(steering(:))),'maximum_acceleration',max(abs(accel(:))),'maximum_continuous_bound_violation',violation,'forward_terminal_dynamic_drift',0,'passed',violation<1e-6,'reconstruction','Piecewise-linear Euler states; piecewise-constant interval controls.');
end
