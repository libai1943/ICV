function [reference,diagnostic]=prepare_reference(scene,sfm)
% Route-reservation fallback makes the initializer endpoint-compatible.
% It is declared separately from SFM: local repulsion does not resolve deadlock.
count=size(scene.start,2);nodes=scene.nodes;time=linspace(0,scene.horizon,nodes);fine=linspace(0,scene.horizon,4*nodes);offset=(scene.geometry.LW+scene.geometry.LF-scene.geometry.LR)/2;
radius=hypot(scene.geometry.LW+scene.geometry.LF+scene.geometry.LR,scene.geometry.LB)/2;reference=zeros(10,nodes,count);occupied=zeros(2,numel(fine),count);delays=zeros(1,count);
for car=1:count
 route=scene.routes{car};arc=[0,cumsum(vecnorm(diff(route,1,2)))];duration=max(30,1.9*arc(end)/3.8);found=false;
 for delay=0:1:(scene.horizon-duration-1)
  z=timed_route(route,arc,fine,delay,duration,scene,car);centers=z(1:2,:)+offset*[cos(z(3,:));sin(z(3,:))];
  safe=true;
  for other=1:car-1
   if min(vecnorm(centers-occupied(:,:,other)))<2*radius+scene.margin+.25,safe=false;break;end
  end
  if safe,found=true;break;end
 end
 assert(found,'Route reservation exhausted the finite horizon; increase scene.horizon.');
 delays(car)=delay;occupied(:,:,car)=centers;z=timed_route(route,arc,time,delay,duration,scene,car);
 reference(1:6,:,car)=z;reference(7,:,car)=[diff(z(6,:))/mean(diff(time)),0];reference(8,:,car)=[diff(z(5,:))/mean(diff(time)),0];
 reference(9:10,:,car)=z(1:2,:)+offset*[cos(z(3,:));sin(z(3,:))];
end
diagnostic=struct('sfm_terminal_position_errors',vecnorm(squeeze(sfm(1:2,end,:))-scene.goal(1:2,:)),'fallback','Deterministic route-reservation initializer; original SFM proposal retained for inspection.','entry_delays',delays);
end
function state=timed_route(route,arc,time,delay,duration,scene,car)
fraction=min(1,max(0,(time-delay)/duration));progress=(10*fraction.^3-15*fraction.^4+6*fraction.^5)*arc(end);
xy=interp1(arc,route',progress,'pchip')';spacing=mean(diff(time));velocity=[gradient(xy(1,:),spacing);gradient(xy(2,:),spacing)];speed=vecnorm(velocity);angle=unwrap(atan2(velocity(2,:),velocity(1,:)));
active=speed>.02;if any(active),angle(1:find(active,1)-1)=angle(find(active,1));angle(find(active,1,'last')+1:end)=angle(find(active,1,'last'));end
angle=unwrap(angle);angle=angle+2*pi*round((scene.start(3,car)-angle(1))/(2*pi));
curvature=gradient(angle)/spacing./max(speed,.05);steering=atan(scene.geometry.LW*curvature);steering(speed<.03)=0;
acceleration=gradient(speed)/spacing;state=[xy;angle;speed;steering;acceleration];state(:,1)=scene.start(:,car);state(:,end)=scene.goal(:,car);
end
