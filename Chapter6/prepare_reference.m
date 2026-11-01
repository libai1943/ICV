function reference=prepare_reference(task,nodes)
% The same independently optimized initial trajectories feed the cooperative solver.
reference=seed_trajectory(task,nodes);count=size(reference.state,3);individual=cell(1,count);longest=0;
for car=1:count
 single=task;single.start=task.start(:,car);single.goal=task.goal(:,car);if isfield(task,'transit_normals'),single.transit_normals=task.transit_normals(:,car);end;seed=struct('state',reference.state(:,:,car),'control',reference.control(:,:,car),'horizon',reference.horizon);
 [d,~]=collision_vector(seed.state,single);
 [sol,info]=solve_reduced(single,seed,true(numel(d),1),45);
 if info.success,individual{car}=sol;else,individual{car}=seed;end
 longest=max(longest,individual{car}.horizon);
 fprintf('Independent route %d/%d: %s\n',car,count,info.status);
end
% Common temporal initialization: the cooperative solve receive exactly this reference.
% This is only an initial guess; full Euler and collision validation is still required.
orders=[1:count;count:-1:1;[1:2:count,2:2:count];[2:2:count,1:2:count]];
best=inf;bestDelay=zeros(1,count);bestHorizon=min(70,longest*1.12);
for row=1:size(orders,1)
 for spacing=[0,1.3,1.8,2.3,2.8,3.3]
  delay=zeros(1,count);delay(orders(row,:))=(0:count-1)*spacing;
  horizon=max(cellfun(@(r)r.horizon,individual)+delay);if horizon>70,continue;end
  time=linspace(0,horizon,181);q=zeros(6,numel(time),count);
  for car=1:count
   r=individual{car};at=max(0,min(time-delay(car),r.horizon));
   q(:,:,car)=interp1(linspace(0,r.horizon,nodes),r.state',at,'linear')';
  end
  [d2,radii]=collision_vector(q,task);gap=sqrt(max(d2,0))-radii;
  score=sum(max(0,task.margin+.12-gap).^2,'all')+.0001*horizon;
  if score<best,best=score;bestDelay=delay;bestHorizon=horizon;end
 end
end
reference.horizon=bestHorizon;target=linspace(0,bestHorizon,nodes);
for car=1:count
 r=individual{car};old=linspace(0,r.horizon,nodes);t=max(0,min(target-bestDelay(car),r.horizon));
 reference.state(:,:,car)=interp1(old,r.state',t,'linear')';
 for k=1:nodes-1
  index=min(nodes-1,max(1,floor(t(k)/r.horizon*(nodes-1))+1));reference.control(:,k,car)=r.control(:,index);
  if target(k)<bestDelay(car)||target(k)>=bestDelay(car)+r.horizon,reference.control(:,k,car)=0;end
 end
end
reference.initialization=struct('method','obstacle-aware independent routes and common temporal staggering','delays',bestDelay,'screening_score',best,'validated_plan',false);
fprintf('Common temporal seed: horizon %.2f s, screening score %.5g\n',bestHorizon,best);
end
