function report=validate_trajectories(result)
scene=result.scene;q=result.trajectory;count=size(q,3);nodes=scene.nodes;dt=scene.horizon/(nodes-1);substeps=80;g=scene.geometry;[radius,offsets]=disc_geometry(scene);nd=numel(offsets);
dense=zeros(6,(nodes-1)*substeps+1,count);defect=0;boundary=0;
for car=1:count
 defect=max(defect,max(abs(q(1:6,2:end,car)-euler_step(q(1:6,1:end-1,car),q(7:8,1:end-1,car),dt,g.LW)),[],'all'));
 dense(:,:,car)=interp1(0:nodes-1,q(1:6,:,car)',linspace(0,nodes-1,(nodes-1)*substeps+1),'linear')';
 boundary=max([boundary;abs(q(1:6,1,car)-scene.start(:,car));abs(q(1:6,end,car)-scene.goal(:,car))]);
end
centers=disc_centers(dense,scene);clearance=inf(1,size(dense,2));
for car=1:count
 for disk=1:nd
  for other=car+1:count
   for second=1:nd,clearance=min(clearance,vecnorm(centers(:,:,disk,car)-centers(:,:,second,other))-2*radius);end
  end
  for obstacle=1:numel(scene.obstacles),gap=point_polygon_distance(centers(:,:,disk,car)',scene.obstacles{obstacle})'-radius;clearance=min(clearance,gap);end
 end
end
motion=diff(q(1:3,:,:),1,2)/dt;speedBound=max(vecnorm(motion(1:2,:,:))+max(abs(offsets))*abs(motion(3,:,:)),[],'all');bound=min(clearance)-speedBound*dt/substeps;
bounds=max([0;reshape(-q(4,:,:),[],1);reshape(q(4,:,:)-scene.maxSpeed,[],1);reshape(abs(q(5,:,:))-scene.maxSteering,[],1);reshape(abs(q(6,:,:))-scene.maxAcceleration,[],1);reshape(abs(q(7,:,:))-scene.maxJerk,[],1);reshape(abs(q(8,:,:))-scene.maxSteeringRate,[],1)]);
centerBody=squeeze(mean(centers,3));inside=squeeze((centerBody(1,:,:)/scene.central_axes(1)).^2+(centerBody(2,:,:)/scene.central_axes(2)).^2)<=1;concurrency=sum(inside,2);
report=struct('maximum_euler_defect',defect,'maximum_interpolation_node_error',0,'boundary_violation',boundary,'bound_violation',bounds,'minimum_sampled_clearance',min(clearance),'interval_clearance_bound',bound,'clearance',clearance,'maximum_simultaneous_central_occupancy',max(concurrency),'maximum_steering_rad',max(abs(q(5,:,:)),[],'all'),'disc_count',nd,'interpolation_subdivisions',substeps,'disc_radius_m',radius,'passed',defect<1e-5&&boundary<1e-5&&bounds<1e-6&&bound>=0);
end
