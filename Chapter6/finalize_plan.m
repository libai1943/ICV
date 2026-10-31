function result=finalize_plan(result)
% Common fleet-wide time dilation preserves geometry and relative timing.
% This postprocessor is valid for the stopped endpoints of these benchmark cases.
if isfield(result,'postprocessing'),return;end
clock=tic;before=audit_control_bounds(result.state,result.control,result.horizon,result.task,false);scale=1;originalHorizon=result.horizon;
if result.success && ~before.passed
 task=result.task;speed=max(abs([before.minimum_velocity,before.maximum_velocity]));
 scale=max([1,speed/task.maxSpeed,sqrt(before.maximum_acceleration/task.maxAcceleration),(max(abs(result.control(1,:,:)),[],'all')/task.maxJerk)^(1/3),max(abs(result.control(2,:,:)),[],'all')/task.maxSteeringRate]);
 if scale>1,scale=scale*(1+1e-7);end
 assert(max(abs(result.state(4:6,[1,end],:)),[],'all')<1e-5,'Time dilation assumes stopped endpoint states.');
 result.state(4,:,:)=result.state(4,:,:)/scale;result.state(6,:,:)=result.state(6,:,:)/scale^2;
 result.control(1,:,:)=result.control(1,:,:)/scale^3;result.control(2,:,:)=result.control(2,:,:)/scale;result.horizon=result.horizon*scale;
 result.validation=validate_plan(result,result.task);
end
result.continuous_control_audit=audit_control_bounds(result.state,result.control,result.horizon,result.task,false);
result.success=result.success&&result.validation.passed&&result.continuous_control_audit.passed&&result.horizon<=75;
dt=result.horizon/(size(result.state,2)-1);result.final_cost=result.horizon+.01*dt*sum(result.state(6,1:end-1,:).^2+result.state(4,1:end-1,:).^2.*result.control(2,:,:).^2,'all');
result.postprocessing=struct('method','common fleet-wide time dilation','scale',scale,'original_horizon',originalHorizon,'pre_repair_bound_violation',before.maximum_continuous_bound_violation,'seconds',toc(clock));
result.wall_seconds=result.wall_seconds+result.postprocessing.seconds;
end
