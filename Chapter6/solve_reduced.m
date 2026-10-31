function [candidate,info]=solve_reduced(task,reference,active,mode,budget)
% Fresh six-state forward Euler direct multiple shooting; IPOPT with MA57.
import casadi.*
nodes=size(reference.state,2);count=size(reference.state,3);states=SX.sym('states',6,nodes*count);
inputs=SX.sym('inputs',2,(nodes-1)*count);duration=SX.sym('duration');
step=duration/(nodes-1);equations={};inequalities={};objective=duration;
stages=cell(1,count);midpoints=cell(1,count);
for car=1:count
 q=states(:,(car-1)*nodes+(1:nodes));u=inputs(:,(car-1)*(nodes-1)+(1:nodes-1));stages{car}=q;middle=SX.zeros(6,nodes-1);
 equations{end+1}=reshape(q(:,2:end)-euler_step(q(:,1:end-1),u,step,task.geometry.LW),6*(nodes-1),1);
 middle=euler_step(q(:,1:end-1),u,step/2,task.geometry.LW);
 midpoints{car}=middle;
 equations{end+1}=q(:,1)-[task.start(:,car);0;0;0];equations{end+1}=q(:,end)-[task.goal(:,car);0;0;0];
 objective=objective+.01*step*(sum1(q(6,1:end-1)'.^2)+sum1((q(4,1:end-1)'.*u(2,:)').^2));
end
% CasADi has no native 3D symbolic arrays: construct collision terms explicitly.
[d2,radii]=symbolic_distances(stages,task);[dm,~]=symbolic_distances(midpoints,task);
threshold=(radii+task.margin).^2;
allExact=d2-repmat(threshold,nodes,1);
allMid=dm-repmat(threshold,nodes-1,1);
flat=allExact(:);flatMid=allMid(:);
selection=reshape(active,nodes,[]);middleSelection=selection(1:end-1,:)|selection(2:end,:);
if strcmp(mode,'linearized')
 geometryVariables=[states(:);inputs(:);duration];both=[flat;flatMid];
 derivative=jacobian(both,geometryVariables);evaluator=Function('linearize',{geometryVariables},{both,derivative});
 starting=[reference.state(:);reference.control(:);reference.horizon];[values,jac]=evaluator(starting);
 linear=values+jac*(geometryVariables-starting);
 selected=[active;middleSelection(:)];inequalities{end+1}=linear(find(selected));
else
 inequalities{end+1}=flat(find(active));inequalities{end+1}=flatMid(find(middleSelection(:)));
end
eq=vertcat(equations{:});ineq=vertcat(inequalities{:});constraints=[eq;ineq];
variables=[states(:);inputs(:);duration];
lower=repmat([-20;-20;-100;-task.maxSpeed;-task.maxSteering;-task.maxAcceleration],nodes*count,1);
upper=repmat([20;20;100;task.maxSpeed;task.maxSteering;task.maxAcceleration],nodes*count,1);
lower=[lower;repmat([-task.maxJerk;-task.maxSteeringRate],(nodes-1)*count,1);5];
upper=[upper;repmat([task.maxJerk;task.maxSteeringRate],(nodes-1)*count,1);75];
options=struct('print_time',false);options.ipopt=struct('print_level',0,'sb','yes','max_iter',1500,'max_cpu_time',budget,'tol',1e-7,'acceptable_tol',1e-5,'acceptable_iter',8,'ma57_pivot_order',2,'ma57_pre_alloc',3,'linear_solver','ma57','hsllib',getenv('IPOPT_HSL_LIBRARY'),'bound_relax_factor',0,'mu_strategy','adaptive');
solver=nlpsol('trajectory','ipopt',struct('x',variables,'f',objective,'g',constraints),options);
model=struct('solver',solver,'lower',lower,'upper',upper,'equations',numel(eq),'inequalities',numel(ineq),'nodes',nodes,'count',count,'start',task.start,'goal',task.goal,'linearized',strcmp(mode,'linearized'));
[candidate,info]=execute(model,reference,active);
end
function [candidate,info]=execute(model,reference,active)
nodes=model.nodes;count=model.count;
selection=reshape(active,nodes,[]);mid=selection(1:end-1,:)|selection(2:end,:);selected=[active;mid(:)];
low=zeros(model.inequalities,1);
guess=[reference.state(:);reference.control(:);reference.horizon];
timer=tic;sol=model.solver('x0',guess,'lbx',model.lower,'ubx',model.upper,'lbg',[zeros(model.equations,1);low],'ubg',[zeros(model.equations,1);inf(model.inequalities,1)]);
elapsed=toc(timer);stats=model.solver.stats();answer=full(sol.x);
candidate.state=reshape(answer(1:6*nodes*count),6,nodes,count);candidate.control=reshape(answer(6*nodes*count+1:end-1),2,nodes-1,count);candidate.horizon=answer(end);
residual=full(sol.g);collision=residual(model.equations+1:end);
info=struct('success',logical(stats.success),'status',stats.return_status,'seconds',elapsed,'iterations',stats.iter_count,'cost',full(sol.f),'kinematic_residual',max(abs(residual(1:model.equations))),'loaded_collision_constraints',nnz(selected),'minimum_loaded_constraint',min([inf;collision]));
end

function [values,radii]=symbolic_distances(stages,task)
count=numel(stages);centers=cell(count,2);terms={};radii=[];cover=hypot((task.geometry.LW+task.geometry.LF+task.geometry.LR)/4,task.geometry.LB/2);
for car=1:count,centers(car,:)=disc_centers(stages{car},task.geometry);end
for car=1:count
 for other=car+1:count
  for a=1:2,for b=1:2,delta=centers{car,a}-centers{other,b};terms{end+1}=sum(delta.^2,1)';radii(end+1)=2*cover;end,end
 end
 for obstacle=1:size(task.obstacles,1)
  for a=1:2,delta=centers{car,a}-task.obstacles(obstacle,1:2)';terms{end+1}=sum(delta.^2,1)';radii(end+1)=cover+task.obstacles(obstacle,3);end
 end
end
values=horzcat(terms{:});
end
