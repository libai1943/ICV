function reference=prepare_reference(task,nodes)
% The same independently optimized initial trajectories feed every comparator.
reference=seed_trajectory(task,nodes);count=size(reference.state,3);individual=cell(1,count);longest=0;
for car=1:count
 single=task;single.start=task.start(:,car);single.goal=task.goal(:,car);seed=struct('state',reference.state(:,:,car),'control',reference.control(:,:,car),'horizon',reference.horizon);
 [d,~]=collision_vector(seed.state,single);
 [sol,info]=solve_reduced(single,seed,true(numel(d),1),'exact',20);
 if info.success,individual{car}=sol;else,individual{car}=seed;end
 longest=max(longest,individual{car}.horizon);
 fprintf('Independent route %d/%d: %s\n',car,count,info.status);
end
reference.horizon=longest*1.12;target=linspace(0,reference.horizon,nodes);
for car=1:count
 r=individual{car};old=linspace(0,r.horizon,nodes);t=min(target,r.horizon);
 reference.state(:,:,car)=interp1(old,r.state',t,'linear')';
 for k=1:nodes-1
  index=min(nodes-1,max(1,floor(t(k)/r.horizon*(nodes-1))+1));reference.control(:,k,car)=r.control(:,index);if target(k)>=r.horizon,reference.control(:,k,car)=0;end
 end
end
end
