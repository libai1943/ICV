function report=validate_plan(candidate,task)
% Native Euler residuals and full geometry of the linear pose reconstruction.
q=candidate.state;u=candidate.control;nodes=size(q,2);count=size(q,3);dt=candidate.horizon/(nodes-1);substeps=40;
dense=zeros(6,(nodes-1)*substeps+1,count);residual=0;drift=0;terminal=0;
for car=1:count
 dense(:,1,car)=q(:,1,car);
 for k=1:nodes-1
  trial=q(:,k,car);residual=max(residual,max(abs(q(:,k+1,car)-euler_step(trial,u(:,k,car),dt,task.geometry.LW))));
  for part=1:substeps
   idx=(k-1)*substeps+part;
   dense(:,idx+1,car)=q(:,k,car)+(part/substeps)*(q(:,k+1,car)-q(:,k,car));
  end
 end
 drift=max(drift,max(abs(dense(1:3,1:substeps:end,car)-q(1:3,:,car)),[],'all'));
 terminal=max([terminal;abs(q(:,1,car)-[task.start(:,car);0;0;0]);abs(q(:,end,car)-[task.goal(:,car);0;0;0])]);
end
[squared,radius]=collision_vector(dense,task);clearance=sqrt(max(0,squared))-radius;
% Global derivative bound covers both moving endpoints of any disc pair.
offset=-task.geometry.LR+3*(task.geometry.LW+task.geometry.LF+task.geometry.LR)/4;
speedBound=task.maxSpeed*(1+abs(offset)*tan(task.maxSteering)/task.geometry.LW);
motion=diff(q(1:3,:,:),1,2)/dt;speedBound=max(vecnorm(motion(1:2,:,:))+abs(offset)*abs(motion(3,:,:)),[],'all');bound=min(clearance(:))-speedBound*dt/substeps; % 2*speedBound*half subinterval.
bounds=max([0;reshape(abs(q(4,:,:))-task.maxSpeed,[],1);reshape(abs(q(5,:,:))-task.maxSteering,[],1);reshape(abs(q(6,:,:))-task.maxAcceleration,[],1);reshape(abs(u(1,:,:))-task.maxJerk,[],1);reshape(abs(u(2,:,:))-task.maxSteeringRate,[],1)]);
report=struct('minimum_sampled_clearance',min(clearance(:)),'interval_clearance_bound',bound,'maximum_euler_defect',residual,'maximum_interpolation_node_error',drift,'boundary_violation',terminal,'bound_violation',bounds,'passed',bound>=0 && residual<1e-5 && terminal<1e-5 && bounds<1e-6 && drift<.03);
end
