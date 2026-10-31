function result=optimize_trajectories(scene,reference)
import casadi.*
count=size(reference,3);steps=scene.nodes;dt=scene.horizon/(steps-1);offset=(scene.geometry.LW+scene.geometry.LF-scene.geometry.LR)/2;history=struct([]);timer=tic;
for iteration=1:6
 corridor=construct_corridors(scene,reference);pairs=size(corridor.pairs,1);total=10*steps*count;
 variables=SX.sym('trajectory',total+pairs*steps,1);residuals={};linear={};lo=[];hi=[];normal=SX(0);
 lower=-inf(size(variables));upper=inf(size(variables));guess=[reference(:);zeros(pairs*steps,1)];
 for car=1:count
  indices=(car-1)*10*steps+(1:10*steps);q=reshape(variables(indices),10,steps);lb=-inf(10,steps);ub=-lb;
  lb(4,:)=0;ub(4,:)=scene.maxSpeed;lb(5,:)=-scene.maxSteering;ub(5,:)=scene.maxSteering;lb(6,:)=-scene.maxAcceleration;ub(6,:)=scene.maxAcceleration;
  lb(7,:)=-scene.maxJerk;ub(7,:)=scene.maxJerk;lb(8,:)=-scene.maxSteeringRate;ub(8,:)=scene.maxSteeringRate;
  box=corridor.absolute(:,:,car);lb(9:10,:)=box([1,3],:);ub(9:10,:)=box([2,4],:);
  lb(1:6,1)=scene.start(:,car);ub(1:6,1)=scene.start(:,car);lb(1:6,end)=scene.goal(:,car);ub(1:6,end)=scene.goal(:,car);
  lb(7:8,end)=0;ub(7:8,end)=0;
  lower(indices)=lb(:);upper(indices)=ub(:);
  residuals{end+1}=reshape(q(1:6,2:end)-euler_step(q(1:6,1:end-1),q(7:8,1:end-1),dt,scene.geometry.LW),6*(steps-1),1);
  residuals{end+1}=reshape(q(9:10,:)-q(1:2,:)-offset*[cos(q(3,:));sin(q(3,:))],2*steps,1);
  normal=normal+.02*dt*(sumsqr(q(6,:))+sumsqr(q(8,:)))+.002*dt*sumsqr(q(7,:));
 end
 for pair=1:pairs
  cars=corridor.pairs(pair,:);a=reshape(variables((cars(1)-1)*10*steps+(1:10*steps)),10,steps);b=reshape(variables((cars(2)-1)*10*steps+(1:10*steps)),10,steps);
  indices=total+(pair-1)*steps+(1:steps);slack=variables(indices)';lower(indices)=0;if iteration==6,upper(indices)=0;end
  delta=a(9:10,:)-b(9:10,:);box=corridor.relative(:,:,pair);
  linear{end+1}=reshape([delta(1,:)+slack;delta(1,:)-slack;delta(2,:)+slack;delta(2,:)-slack],4*steps,1);
  lo=[lo;reshape([box(1,:);-inf(1,steps);box(3,:);-inf(1,steps)],[],1)];hi=[hi;reshape([inf(1,steps);box(2,:);inf(1,steps);box(4,:)],[],1)];
 end
 residual=vertcat(residuals{:});constraints=vertcat(linear{:});weight=10^(2*iteration);penalty=sumsqr(residual);slackPenalty=100*sum1(variables(total+1:end));
 options=struct('print_time',false);options.ipopt=struct('print_level',0,'sb','yes','max_iter',900,'max_cpu_time',90,'tol',1e-8,'acceptable_iter',0,'linear_solver','ma57','hsllib',getenv('IPOPT_HSL_LIBRARY'),'bound_relax_factor',0,'mu_strategy','adaptive');
 if iteration<6
  cost=penalty+slackPenalty+normal/weight;
  jac=jacobian(residual,variables);curvature=2*(jac'*jac)+hessian(normal,variables)/weight;
  sigma=SX.sym('sigma');multipliers=SX.sym('multipliers',numel(constraints));
  options.hess_lag=Function('gn',{variables,SX.sym('parameters',0),sigma,multipliers},{triu(sigma*curvature)});
 else
  cost=normal;constraints=[constraints;residual];lo=[lo;zeros(numel(residual),1)];hi=[hi;zeros(numel(residual),1)];
 end
 fprintf('LIOM iteration %d: %d variables, %d relative pairs\n',iteration,numel(variables),pairs);
 solver=nlpsol('liom','ipopt',struct('x',variables,'f',cost,'g',constraints),options);solution=solver('x0',guess,'lbx',lower,'ubx',upper,'lbg',lo,'ubg',hi);
 stats=solver.stats();value=full(solution.x);proposed=reshape(value(1:total),10,steps,count);
 measure=Function('residual',{variables},{residual});defect=max(abs(full(measure(value))));slack=max([0;value(total+1:end)]);
 item=struct('iteration',iteration,'penalty_weight',weight,'maximum_defect',defect,'maximum_slack',slack,'status',stats.return_status,'success',logical(stats.success),'loaded_pairs',pairs,'repaired_absolute_seeds',corridor.repaired_absolute,'repaired_relative_seeds',corridor.repaired_relative);
 history=[history,item];if stats.success,reference=proposed;end;fprintf('defect %.3g, slack %.3g, %s\n',defect,slack,stats.return_status);
 if stats.success && defect<1e-5 && slack<1e-6
  result=struct('scene',scene,'trajectory',reference,'history',history,'wall_seconds',toc(timer));result.validation=validate_trajectories(result);result.success=result.validation.passed;
  if result.success,return;end
 end
 if iteration==6 && ~stats.success,fprintf('Restoration did not converge; checking last accepted iterate.\n');end
end
result=struct('scene',scene,'trajectory',reference,'history',history,'wall_seconds',toc(timer));result.validation=validate_trajectories(result);result.success=result.validation.passed;
end
