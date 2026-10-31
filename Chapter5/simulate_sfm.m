function reference=simulate_sfm(scene)
% Social force model with velocity relaxation, anticipatory repulsion,
% bounded jerk/steering rate, and bicycle propagation. It is an initializer.
count=size(scene.start,2);steps=scene.nodes;dt=scene.horizon/(steps-1);
trajectory=zeros(10,steps,count);trajectory(1:6,1,:)=reshape(scene.start,6,1,count);progress=ones(1,count);
offset=(scene.geometry.LW+scene.geometry.LF-scene.geometry.LR)/2;radius=hypot(scene.geometry.LW+scene.geometry.LF+scene.geometry.LR,scene.geometry.LB)/2;
for k=1:steps-1
 for car=1:count
  q=trajectory(1:6,k,car);route=scene.routes{car};allowed=progress(car):min(size(route,2),progress(car)+75);
  [~,nearest]=min(sum((route(:,allowed)-q(1:2)).^2,1));progress(car)=allowed(nearest);
  arc=[0,cumsum(vecnorm(diff(route(:,progress(car):end),1,2)))];look=find(arc>=5.5,1);if isempty(look),look=numel(arc);end
  guide=route(:,progress(car)+look-1);remaining=arc(end);desired=min(3.6,sqrt(max(0,1.4*remaining)));if remaining<.7,desired=0;end
  direction=guide-q(1:2);direction=direction/max(norm(direction),1e-6);body=[cos(q(3));sin(q(3))];force=(desired*direction-q(4)*body)/1.4;
  center=q(1:2)+offset*body;
  for other=1:count
   if other==car,continue;end
   z=trajectory(1:6,k,other);otherBody=[cos(z(3));sin(z(3))];delta=center-z(1:2)-offset*otherBody;
   relativeVelocity=q(4)*body-z(4)*otherBody;tau=max(0,min(1.2,-dot(delta,relativeVelocity)/max(dot(relativeVelocity,relativeVelocity),1e-6)));
   predicted=delta+tau*relativeVelocity;distance=max(norm(predicted),.1);
   force=force+2.5*exp(min(8,(2*radius+scene.margin-distance)/1.3))*predicted/distance;
  end
  for obstacle=1:numel(scene.obstacles)
   polygon=scene.obstacles{obstacle};ends=polygon([2:end,1],:);edges=ends-polygon;fractions=max(0,min(1,sum((center'-polygon).*edges,2)./sum(edges.^2,2)));points=polygon+fractions.*edges;
   [distance,index]=min(vecnorm(points-center',2,2));away=center-points(index,:)';away=away/max(distance,.05);
   force=force+3*exp(min(8,(radius+scene.margin-distance)/.8))*away;
  end
  desiredAcceleration=min(scene.maxAcceleration,max(-scene.maxAcceleration,dot(force,body)));
  headingError=atan2(sin(atan2(force(2)+desired*body(2),force(1)+desired*body(1))-q(3)),cos(atan2(force(2)+desired*body(2),force(1)+desired*body(1))-q(3)));
  desiredSteering=min(scene.maxSteering,max(-scene.maxSteering,atan(2*scene.geometry.LW*sin(headingError)/5.5)));
  input=[min(scene.maxJerk,max(-scene.maxJerk,(desiredAcceleration-q(6))/dt));min(scene.maxSteeringRate,max(-scene.maxSteeringRate,(desiredSteering-q(5))/dt))];
  next=euler_step(q,input,dt,scene.geometry.LW);
  if next(4)<0,next(4)=0;next(6)=0;end
  if next(4)>scene.maxSpeed,next(4)=scene.maxSpeed;next(6)=0;end
  trajectory(7:8,k,car)=input;trajectory(1:6,k+1,car)=next;
 end
end
for car=1:count,q=trajectory(1:6,:,car);trajectory(9:10,:,car)=q(1:2,:)+offset*[cos(q(3,:));sin(q(3,:))];end
reference=trajectory;
end
