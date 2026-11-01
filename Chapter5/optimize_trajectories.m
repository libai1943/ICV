function result=optimize_trajectories(scene,reference,resume)
import casadi.*
[~,offsets]=disc_geometry(scene);nd=numel(offsets);rows=8+2*nd;count=size(reference,3);steps=scene.nodes;dt=scene.horizon/(steps-1);history=struct([]);elapsed=0;firstIteration=1;if nargin>2,assert(resume.history(end).iterate_accepted);history=resume.history;elapsed=resume.wall_seconds;firstIteration=numel(history)+1;end;timer=tic;
centers=disc_centers(reference,scene);reference=cat(1,reference(1:8,:,:),zeros(2*nd,steps,count));
for car=1:count,for d=1:nd,reference(7+2*d:8+2*d,:,car)=centers(:,:,d,car);end;end
for iteration=firstIteration:7
 restoration=iteration==7||(iteration>2&&history(end).maximum_defect<1e-3&&history(end).maximum_slack<1e-6);
 corridor=construct_corridors(scene,reference);pairs=size(corridor.pairs,1);guards=size(corridor.guards,1);total=rows*steps*count;
 variables=SX.sym('trajectory',total+pairs+guards,1);residuals={};linear={};lo=[];hi=[];normal=SX(0);Q=cell(1,count);
 lower=-inf(size(variables));upper=inf(size(variables));guess=[reference(:);zeros(pairs+guards,1)];
 for car=1:count
  indices=(car-1)*rows*steps+(1:rows*steps);q=reshape(variables(indices),rows,steps);Q{car}=q;lb=-inf(rows,steps);ub=-lb;
  lb(4,:)=0;ub(4,:)=scene.maxSpeed;lb(5,:)=-scene.maxSteering;ub(5,:)=scene.maxSteering;lb(6,:)=-scene.maxAcceleration;ub(6,:)=scene.maxAcceleration;
  lb(7,:)=-scene.maxJerk;ub(7,:)=scene.maxJerk;lb(8,:)=-scene.maxSteeringRate;ub(8,:)=scene.maxSteeringRate;
  for d=1:nd
   box=corridor.absolute(:,:,d,car);rr=7+2*d:8+2*d;lb(rr,:)=box([1,3],:);ub(rr,:)=box([2,4],:);
   residuals{end+1}=reshape(q(rr,:)-q(1:2,:)-offsets(d)*[cos(q(3,:));sin(q(3,:))],2*steps,1);
  end
  lb(1:6,1)=scene.start(:,car);ub(1:6,1)=scene.start(:,car);lb(1:6,end)=scene.goal(:,car);ub(1:6,end)=scene.goal(:,car);lb(7:8,end)=0;ub(7:8,end)=0;lower(indices)=lb(:);upper(indices)=ub(:);
  residuals{end+1}=reshape(q(1:6,2:end)-euler_step(q(1:6,1:end-1),q(7:8,1:end-1),dt,scene.geometry.LW),6*(steps-1),1);
  linear{end+1}=(q(3,2:end)-q(3,1:end-1))';lo=[lo;-corridor.yaw_bound(1,:,car)'];hi=[hi;corridor.yaw_bound(1,:,car)'];
  normal=normal+.02*dt*(sumsqr(q(6,:))+sumsqr(q(8,:)))+.002*dt*sumsqr(q(7,:));
 end
 for pair=1:pairs
  p=corridor.pairs(pair,:);a=Q{p(1)};b=Q{p(3)};k=p(5);slack=variables(total+pair);lower(total+pair)=0;if restoration,upper(total+pair)=0;end
  delta=a(7+2*p(2):8+2*p(2),k)-b(7+2*p(4):8+2*p(4),k);box=corridor.relative(:,pair);
  linear{end+1}=[delta(1)+slack;delta(1)-slack;delta(2)+slack;delta(2)-slack];lo=[lo;box(1);-inf;box(3);-inf];hi=[hi;inf;box(2);inf;box(4)];
 end

 for index=1:guards
  p=corridor.guards(index,:);a=Q{p(1)};b=Q{p(3)};k=p(5);axisNormal=corridor.guard_normal(:,index);limit=corridor.guard_limit(index);
  slack=variables(total+pairs+index);lower(total+pairs+index)=0;if restoration,upper(total+pairs+index)=0;end
  delta=a(7+2*p(2):8+2*p(2),k:k+1)-b(7+2*p(4):8+2*p(4),k:k+1);
  linear{end+1}=(axisNormal'*delta+slack)';lo=[lo;limit;limit];hi=[hi;inf;inf];
 end
 residual=vertcat(residuals{:});constraints=vertcat(linear{:});weight=10^(2*min(iteration,5));penalty=sumsqr(residual);slackPenalty=10*sum1(variables(total+1:end));
 options=struct('print_time',false);options.ipopt=struct('print_level',0,'sb','yes','max_iter',900,'max_cpu_time',180,'tol',1e-11,'compl_inf_tol',1e-12,'acceptable_iter',0,'linear_solver','ma57','hsllib',getenv('IPOPT_HSL_LIBRARY'),'ma57_pivot_order',2,'ma57_pre_alloc',3,'bound_relax_factor',0,'mu_strategy','adaptive');
 if ~restoration
  cost=penalty+slackPenalty+normal/weight;jac=jacobian(residual,variables);curvature=2*(jac'*jac)+hessian(normal,variables)/weight+1e-7*SX.eye(numel(variables));
  sigma=SX.sym('sigma');multipliers=SX.sym('multipliers',numel(constraints));options.hess_lag=Function('gn',{variables,SX.sym('parameters',0),sigma,multipliers},{triu(sigma*curvature)});
 else
  cost=normal;constraints=[constraints;residual];lo=[lo;zeros(numel(residual),1)];hi=[hi;zeros(numel(residual),1)];
 end
 fprintf('LIOM iteration %d: %d variables, %d active disc-pair-time tuples\n',iteration,numel(variables),pairs);
 solver=nlpsol('liom','ipopt',struct('x',variables,'f',cost,'g',constraints),options);solution=solver('x0',guess,'lbx',lower,'ubx',upper,'lbg',lo,'ubg',hi);
 stats=solver.stats();value=full(solution.x);proposed=reshape(value(1:total),rows,steps,count);measure=Function('residual',{variables},{residual});defect=max(abs(full(measure(value))));slack=max([0;value(total+1:end)]);
 hard=max([0;lower-value;value-upper;lo-full(solution.g);full(solution.g)-hi]);accepted=all(isfinite(value))&&hard<1e-6;
 item=struct('iteration',iteration,'solver_tolerance',options.ipopt.tol,'restoration',restoration,'penalty_weight',weight,'maximum_defect',defect,'maximum_slack',slack,'status',stats.return_status,'success',logical(stats.success),'iterate_accepted',accepted,'hard_constraint_violation',hard,'loaded_pairs',pairs,'interval_guards',guards,'repaired_absolute_seeds',corridor.repaired_absolute,'repaired_relative_seeds',corridor.repaired_relative);history=[history,item];
 checkpoint=struct('scene',scene,'trajectory',proposed,'history',history,'wall_seconds',elapsed+toc(timer));save(fullfile(fileparts(mfilename('fullpath')),sprintf('output/checkpoint_%02d.mat',count)),'checkpoint');
 if accepted,reference=proposed;end
 fprintf('defect %.3g, slack %.3g, hard %.3g, %s\n',defect,slack,hard,stats.return_status);
 if accepted&&defect<1e-5&&slack<1e-6
  result=struct('scene',scene,'trajectory',reference,'history',history,'wall_seconds',elapsed+toc(timer));result.validation=validate_trajectories(result);result.success=result.validation.passed;fprintf('Validation: defect %.3g, bounds %.3g, sampled gap %.4f, interval bound %.4f\n',result.validation.maximum_euler_defect,result.validation.bound_violation,result.validation.minimum_sampled_clearance,result.validation.interval_clearance_bound);
  if result.success,return;end
 end
end
result=struct('scene',scene,'trajectory',reference,'history',history,'wall_seconds',elapsed+toc(timer));result.validation=validate_trajectories(result);result.success=result.validation.passed;fprintf('Validation: defect %.3g, bounds %.3g, sampled gap %.4f, interval bound %.4f\n',result.validation.maximum_euler_defect,result.validation.bound_violation,result.validation.minimum_sampled_clearance,result.validation.interval_clearance_bound);
end
