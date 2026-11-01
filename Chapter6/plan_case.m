function result=plan_case(task,reference,budget)
if nargin<3,budget=480;end
method='asco';
clock=tic;lower=-4;upper=8;history=struct([]);candidate=reference;
[squared,radius]=collision_vector(reference.state,task);
for iteration=1:32
 gap=sqrt(max(0,squared))-radius;
 active=gap>=lower & gap<=upper;
 remaining=budget-toc(clock);if remaining<1,break;end
 try
  callBudget=min(180,remaining);
  [trial,info]=solve_reduced(task,candidate,active(:),callBudget);
 catch exception
  info=struct('success',false,'status',exception.message,'seconds',0,'iterations',0,'cost',NaN,'kinematic_residual',inf,'loaded_collision_constraints',nnz(active),'minimum_loaded_constraint',-inf);trial=candidate;
 end
 info.outer_iteration=iteration;info.lower_threshold=lower;info.upper_threshold=upper;
 history=[history,info];fprintf('%s outer %d: %s (%d constraints, %.2f s)\n',method,iteration,info.status,info.loaded_collision_constraints,info.seconds);
 if info.success
  candidate=trial;validation=validate_plan(candidate,task);
  if validation.passed,break;end
  [squared,radius]=collision_vector(candidate.state,task);lower=max(-4,lower-1.3);upper=upper+.5;
 else
  lower=lower+3;
  if lower>upper,lower=-4;upper=upper+2;end
 end
end
validation=validate_plan(candidate,task);
result=struct('method',method,'case_id',task.id,'state',candidate.state,'control',candidate.control,'horizon',candidate.horizon,'task',task,'history',history,'wall_seconds',toc(clock),'validation',validation,'success',~isempty(history)&&history(end).success&&validation.passed);
result=finalize_plan(result);
end
