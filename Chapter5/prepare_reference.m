function [reference,diagnostic]=prepare_reference(scene,sfm)
% Construct an endpoint-compatible reference from routes and reservations.
% The independent SFM rollout supplies terminal-error diagnostics only.
count=size(scene.start,2);nodes=scene.nodes;time=linspace(0,scene.horizon,nodes);fine=linspace(0,scene.horizon,4*nodes);offset=(scene.geometry.LW+scene.geometry.LF-scene.geometry.LR)/2;
[radius,offsets]=disc_geometry(scene);reference=zeros(8+2*numel(offsets),nodes,count);occupied=zeros(2,numel(fine),numel(offsets),count);delays=zeros(1,count);
for car=1:count
 route=scene.routes{car};arc=[0,cumsum(vecnorm(diff(route,1,2)))];duration=max(30,1.9*arc(end)/3.8);found=false;
 for delay=0:1:(scene.horizon-duration-1)
  z=timed_route(route,arc,fine,delay,duration,scene,car);centers=disc_centers(z,scene);
  safe=true;
  for other=1:car-1
   for da=1:numel(offsets),for db=1:numel(offsets),if min(vecnorm(centers(:,:,da)-occupied(:,:,db,other)))<2*radius+.02,safe=false;end;end;end
   if ~safe,break;end
  end
  if safe,found=true;break;end
 end
 assert(found,'Route reservation exhausted the horizon for ICV %d.',car);
 delays(car)=delay;occupied(:,:,:,car)=centers;z=timed_route(route,arc,time,delay,duration,scene,car);
 reference(1:6,:,car)=z;reference(7,:,car)=[diff(z(6,:))/mean(diff(time)),0];reference(8,:,car)=[diff(z(5,:))/mean(diff(time)),0];
 centers=disc_centers(z,scene);for d=1:numel(offsets),reference(7+2*d:8+2*d,:,car)=centers(:,:,d);end
end
diagnostic=struct('sfm_terminal_position_errors',vecnorm(squeeze(sfm(1:2,end,:))-scene.goal(1:2,:)),'reference_method','Deterministic route reservation; SFM retained for diagnostics.','entry_delays',delays);
end
function state=timed_route(route,arc,time,delay,duration,scene,car)
fraction=min(1,max(0,(time-delay)/duration));progress=(10*fraction.^3-15*fraction.^4+6*fraction.^5)*arc(end);
xy=interp1(arc,route',progress,'pchip')';spacing=mean(diff(time));velocity=[gradient(xy(1,:),spacing);gradient(xy(2,:),spacing)];speed=vecnorm(velocity);angle=unwrap(atan2(velocity(2,:),velocity(1,:)));
active=speed>.02;if any(active),angle(1:find(active,1)-1)=angle(find(active,1));angle(find(active,1,'last')+1:end)=angle(find(active,1,'last'));end
angle=unwrap(angle);angle=angle+2*pi*round((scene.start(3,car)-angle(1))/(2*pi));
curvature=gradient(angle)/spacing./max(speed,.05);steering=atan(scene.geometry.LW*curvature);steering(speed<.03)=0;
acceleration=gradient(speed)/spacing;state=[xy;angle;speed;steering;acceleration];state(:,1)=scene.start(:,car);state(:,end)=scene.goal(:,car);
end
