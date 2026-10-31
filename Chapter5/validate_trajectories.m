function report=validate_trajectories(result)
scene=result.scene;q=result.trajectory;count=size(q,3);nodes=scene.nodes;dt=scene.horizon/(nodes-1);substeps=20;g=scene.geometry;radius=hypot(g.LW+g.LF+g.LR,g.LB)/2;offset=(g.LW+g.LF-g.LR)/2;
dense=zeros(6,(nodes-1)*substeps+1,count);defect=0;drift=0;boundary=0;
for car=1:count
 dense(:,1,car)=q(1:6,1,car);
 for k=1:nodes-1
  defect=max(defect,max(abs(q(1:6,k+1,car)-rk_step(q(1:6,k,car),q(7:8,k,car),dt,g.LW))));
  for sub=1:substeps
   at=(k-1)*substeps+sub;dense(:,at+1,car)=rk_step(dense(:,at,car),q(7:8,k,car),dt/substeps,g.LW);
  end
 end
 drift=max(drift,max(abs(dense(1:3,1:substeps:end,car)-q(1:3,:,car)),[],'all'));
 boundary=max([boundary;abs(q(1:6,1,car)-scene.start(:,car));abs(q(1:6,end,car)-scene.goal(:,car))]);
end
centers=dense(1:2,:,:)+offset*cat(1,cos(dense(3,:,:)),sin(dense(3,:,:)));minimum=inf;clearance=inf(1,size(dense,2));
for car=1:count
 for other=car+1:count,gap=vecnorm(centers(:,:,car)-centers(:,:,other))-2*radius;clearance=min(clearance,gap);end
 for k=1:size(dense,2)
  point=centers(:,k,car);box=point([1,1,2,2])';
  for obstacle=1:numel(scene.obstacles),clearance(k)=min(clearance(k),polygon_box_distance(box,scene.obstacles{obstacle})-radius);end
 end
end
speedBound=scene.maxSpeed*(1+abs(offset)*tan(scene.maxSteering)/g.LW);bound=min(clearance)-speedBound*dt/substeps;
bounds=max([0;reshape(-q(4,:,:),[],1);reshape(q(4,:,:)-scene.maxSpeed,[],1);reshape(abs(q(5,:,:))-scene.maxSteering,[],1);reshape(abs(q(6,:,:))-scene.maxAcceleration,[],1);reshape(abs(q(7,:,:))-scene.maxJerk,[],1);reshape(abs(q(8,:,:))-scene.maxSteeringRate,[],1)]);
report=struct('maximum_rk4_defect',defect,'maximum_forward_pose_drift',drift,'boundary_violation',boundary,'bound_violation',bounds,'minimum_sampled_clearance',min(clearance),'interval_clearance_bound',bound,'clearance',clearance,'passed',defect<1e-5&&drift<.03&&boundary<1e-5&&bounds<1e-6&&bound>=0);
end
