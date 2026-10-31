function result=finalize_plan(result)
% Audit the stated Euler interpolant; do not rescale a solved experiment.
result.continuous_control_audit=audit_control_bounds(result.state,result.control,result.horizon,result.task,false);
result.success=result.success&&result.validation.passed&&result.continuous_control_audit.passed&&result.horizon<=75;
dt=result.horizon/(size(result.state,2)-1);result.final_cost=result.horizon+.01*dt*sum(result.state(6,1:end-1,:).^2+result.state(4,1:end-1,:).^2.*result.control(2,:,:).^2,'all');
result.postprocessing=struct('method','audit only; no trajectory modification','scale',1,'original_horizon',result.horizon,'seconds',0);
result.discretization='first-order explicit forward Euler';result.optimizer='IPOPT';result.linear_solver='MA57';
end
