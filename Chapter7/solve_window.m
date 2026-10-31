function [motion,info]=solve_window(task,reference,final)
% Euler residual refinement with affine supporting planes and IPOPT/MA57.
import casadi.*
timer=tic;N=task.nodes;count=size(reference,3);dt=task.dt;g=task.geometry;offset=task.offset;limit=2*task.radius+task.margin;
width=1.5+.45*(0:N-1)*dt;bar=reference(1:2,:,:)+offset*cat(1,cos(reference(3,:,:)),sin(reference(3,:,:)));
kept=[];nodeSets={};midSets={};midBar=(bar(:,1:end-1,:)+bar(:,2:end,:))/2;
angleBound=dt*task.maxSpeed*tan(task.maxSteering)/g.LW;
allowance=offset*(1-cos(angleBound/2));midWidth=(width(1:end-1)+width(2:end))/2+allowance;
for i=1:count-1,for j=i+1:count
 at=find(vecnorm(max(abs(bar(:,:,i)-bar(:,:,j))-2*width,0))<limit);
 between=find(vecnorm(max(abs(midBar(:,:,i)-midBar(:,:,j))-2*midWidth,0))<limit);
 if ~isempty(at)||~isempty(between),kept=[kept;i,j];nodeSets{end+1}=at;midSets{end+1}=between;end
end,end
z=SX.sym('motion',9*N*count);normalCost=SX(0);residuals={};rows={};low=[];high=[];lb=-inf(9,N,count);ub=-lb;centers=cell(1,count);
for i=1:count
 q=reshape(z((i-1)*9*N+(1:9*N)),9,N);
 lb(4,:,i)=-task.maxSpeed;ub(4,:,i)=task.maxSpeed;lb(5,:,i)=-task.maxSteering;ub(5,:,i)=task.maxSteering;
 lb(6,:,i)=-task.maxAcceleration;ub(6,:,i)=task.maxAcceleration;lb(7,:,i)=-task.maxSteeringRate;ub(7,:,i)=task.maxSteeringRate;
 lb(1:5,1,i)=reference(1:5,1,i);ub(1:5,1,i)=reference(1:5,1,i);lb(4,end,i)=0;ub(4,end,i)=0;lb(6:7,end,i)=0;ub(6:7,end,i)=0;
 rhs=[q(4,1:end-1).*cos(q(3,1:end-1));q(4,1:end-1).*sin(q(3,1:end-1));q(4,1:end-1).*tan(q(5,1:end-1))/g.LW;q(6:7,1:end-1)];
 residuals{end+1}=reshape(q(1:5,2:end)-q(1:5,1:end-1)-dt*rhs,5*(N-1),1);
 centers{i}=q(8:9,:);residuals{end+1}=reshape(centers{i}-q(1:2,:)-offset*[cos(q(3,:));sin(q(3,:))],2*N,1);
 lb(8:9,:,i)=bar(:,:,i)-width;ub(8:9,:,i)=bar(:,:,i)+width;
 goalError=centers{i}(:,end)-task.goal(:,i);normalCost=normalCost+sumsqr(goalError)+.002*dt*sumsqr(q(6:7,:))+.00001*sumsqr(q(1:7,:)-reference(:,:,i));
 if final,lb(8:9,end,i)=task.goal(:,i);ub(8:9,end,i)=task.goal(:,i);end
end
for pair=1:size(kept,1)
 i=kept(pair,1);j=kept(pair,2);at=nodeSets{pair};between=midSets{pair};
 base=bar(:,at,i)-bar(:,at,j);normal=base./vecnorm(base);delta=centers{i}(:,at)-centers{j}(:,at);
 rows{end+1}=sum1(normal.*delta)';low=[low;limit*ones(numel(at),1)];high=[high;inf(numel(at),1)];
 base=midBar(:,between,i)-midBar(:,between,j);normal=base./vecnorm(base);delta=(centers{i}(:,between)+centers{i}(:,between+1)-centers{j}(:,between)-centers{j}(:,between+1))/2;
 rows{end+1}=sum1(normal.*delta)';low=[low;(limit+2*allowance)*ones(numel(between),1)];high=[high;inf(numel(between),1)];
end
residual=vertcat(residuals{:});constraints=vertcat(rows{:});measure=Function('defects',{z},{residual});jac=jacobian(residual,z);guess=cat(1,reference,bar);motion=reference;statuses={};iterations=[];penaltyHistory=struct([]);
for stage=1:5
 weight=10^(2*stage);cost=sumsqr(residual)+normalCost/weight;curvature=2*(jac'*jac)+hessian(normalCost,z)/weight+1e-6*SX.eye(numel(z));
 sigma=SX.sym('sigma');multipliers=SX.sym('multipliers',numel(constraints));
 options=struct('print_time',false);options.ipopt=struct('print_level',0,'sb','yes','max_iter',700,'max_cpu_time',45,'tol',1e-8,'acceptable_iter',0,'ma57_pivot_order',2,'ma57_pre_alloc',3,'linear_solver','ma57','hsllib',getenv('IPOPT_HSL_LIBRARY'),'bound_relax_factor',0,'mu_strategy','adaptive');
 options.hess_lag=Function('gn',{z,SX.sym('parameters',0),sigma,multipliers},{triu(sigma*curvature)});
 solver=nlpsol('window_liom','ipopt',struct('x',z,'f',cost,'g',constraints),options);sol=solver('x0',guess(:),'lbx',lb(:),'ubx',ub(:),'lbg',low,'ubg',high);stats=solver.stats();guess=reshape(full(sol.x),9,N,count);motion=guess(1:7,:,:);defect=max(abs(full(measure(guess(:)))));statuses{end+1}=stats.return_status;iterations(end+1)=stats.iter_count;entry=struct('weight',weight,'status',stats.return_status,'iterations',stats.iter_count,'maximum_residual',defect,'success',logical(stats.success));if stage==1,penaltyHistory=entry;else,penaltyHistory(end+1)=entry;end
 fprintf('Window penalty %.0e: %s, defect %.3g\n',weight,stats.return_status,defect);
 if defect<1e-5
  audit=validate_motion(task,motion);if audit.passed,break;end
 end
end
audit=validate_motion(task,motion);target=squeeze(motion(1:2,end,:))+offset*[cos(reshape(motion(3,end,:),1,count));sin(reshape(motion(3,end,:),1,count))];
info=struct('status',strjoin(statuses,'; '),'restoration_used',false,'solver_success',logical(stats.success),'feasible',audit.passed&&defect<1e-5,'penalty_history',penaltyHistory,'retained_collision_rows',numel(constraints),'maximum_goal_error',max(vecnorm(target-task.goal)),'mean_goal_error',mean(vecnorm(target-task.goal)),'clearance_bound',audit.clearance_bound,'euler_defect',audit.maximum_euler_defect,'selected_pairs',size(kept,1),'all_pairs',count*(count-1)/2,'seconds',toc(timer),'final_equalities',final);
end

