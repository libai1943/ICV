function scene=make_scenario(count)
% Five unequal, curved approaches and a flattened, lane-free meeting area.
% The seed controls geometry, not a solver's success/failure classification.
if nargin<1,count=12;end
rng(170+count,'twister');
scene.geometry=struct('LW',2.8,'LF',.96,'LR',.929,'LB',1.942);
scene.nodes=241;scene.horizon=120;scene.margin=.12;scene.maxSpeed=5;scene.maxAcceleration=1.2;scene.maxJerk=1;scene.maxSteering=.6;scene.maxSteeringRate=.4;
scene.bounds=[-90,90,-90,90];scene.seed=170+count;
angles=([8,65,147,213,286]+3*(rand(1,5)-.5))*pi/180;
widths=[5.6,5.8,5.5,5.9,5.7];bends=[13,-13,14,-12,13]+rand(1,5)-.5;
arms=cell(1,5);left=arms;right=arms;inner=zeros(2,5);tangent=inner;
for j=1:5
 direction=[cos(angles(j));sin(angles(j))];normal=[-direction(2);direction(1)];
 inner(:,j)=[12*cos(angles(j));9*sin(angles(j))];
 s=linspace(0,100,401);shift=bends(j)*(1-exp(-(s/22).^2));
 path=inner(:,j)+direction*s+normal*shift;
 d=[gradient(path(1,:));gradient(path(2,:))];d=d./vecnorm(d);
 n=[-d(2,:);d(1,:)];arms{j}=path;left{j}=path+widths(j)/2*n;right{j}=path-widths(j)/2*n;tangent(:,j)=d(:,1);
end
% Adjacent road edges close each physical land block. The sampled boundary
% retains its curvature; no convex hull is taken across a concave edge.
scene.obstacles=cell(1,5);
for j=1:5
 next=mod(j,5)+1;outerAngles=linspace(angles(j),angles(next)+(next==1)*2*pi,15);
 exterior=180*[cos(outerAngles);sin(outerAngles)];
 scene.obstacles{j}=[left{j}(:,1:8:end),exterior,fliplr(right{next}(:,1:8:end))]';
end
scene.arms=arms;scene.approach_widths=widths;scene.laneOffset=1.27;scene.central_axes=[12,9];scene.start=zeros(6,count);scene.goal=zeros(6,count);scene.routes=cell(1,count);
movements=[1,3;3,1;2,4;4,2;5,2;2,5;3,5;5,3;4,1;1,4;5,1;1,5;2,3;3,2;4,5];outCount=zeros(1,5);inCount=zeros(1,5);scene.movements=movements(1:count,:);
for car=1:count
 a=movements(car,1);b=movements(car,2);queue=inCount(a);inCount(a)=queue+1;
 rank=outCount(b);outCount(b)=rank+1;
 inbound=offset_path(arms{a},scene.laneOffset);outbound=offset_path(arms{b},-scene.laneOffset);ain=[0,cumsum(vecnorm(diff(inbound,1,2)))];aout=[0,cumsum(vecnorm(diff(outbound,1,2)))];
 first=interp1(ain,inbound',linspace(27+6.5*queue,0,201),'pchip')';
 last=interp1(aout,outbound',linspace(0,65-6.5*rank,301),'pchip')';
 t=linspace(0,1,241);best=inf;
 for handle=5:2:15
  control=[inbound(:,1),inbound(:,1)-handle*tangent(:,a),outbound(:,1)-handle*tangent(:,b),outbound(:,1)];
  connector=control*[ (1-t).^3;3*(1-t).^2.*t;3*(1-t).*t.^2;t.^3 ];
  centerline=[first,connector(:,2:end),last(:,2:end)];centerline=centerline(:,[true,vecnorm(diff(centerline,1,2))>1e-8]);
  candidate=center_to_rear(centerline,scene);score=route_score(candidate,scene);
  if score<best,best=score;route=candidate;end
 end
 theta=unwrap(atan2(gradient(route(2,:)),gradient(route(1,:))));
 scene.start(:,car)=[route(:,1);theta(1);0;0;0];scene.goal(:,car)=[route(:,end);theta(end);0;0;0];scene.routes{car}=route;
end
scene.id=sprintf('bidirectional_curved_five_arm_%02d',count);
end

function shifted=offset_path(path,distance)
d=[gradient(path(1,:));gradient(path(2,:))];d=d./vecnorm(d);shifted=path+distance*[-d(2,:);d(1,:)];
end

function rear=center_to_rear(centerline,scene)
% Fine spatial Euler integration of the body-center/rear-axle relation.
arc=[0,cumsum(vecnorm(diff(centerline,1,2)))];sample=linspace(0,arc(end),ceil(arc(end)/.05)+1);
xy=interp1(arc,centerline',sample,'pchip')';d=[gradient(xy(1,:));gradient(xy(2,:))];tangent=unwrap(atan2(d(2,:),d(1,:)));theta=zeros(size(sample));theta(1)=tangent(1);
offset=(scene.geometry.LW+scene.geometry.LF-scene.geometry.LR)/2;
for k=1:numel(sample)-1,theta(k+1)=theta(k)+(sample(k+1)-sample(k))*sin(tangent(k)-theta(k))/offset;end
rear=xy-offset*[cos(theta);sin(theta)];
end
function score=route_score(route,scene)
delta=[gradient(route(1,:));gradient(route(2,:))];theta=unwrap(atan2(delta(2,:),delta(1,:)));phi=atan(scene.geometry.LW*gradient(theta)./vecnorm(delta));
q=[route;theta];centers=disc_centers(q(:,1:5:end),scene);[radius,~]=disc_geometry(scene);gap=inf;
for disk=1:size(centers,3),for obstacle=1:numel(scene.obstacles),gap=min(gap,min(point_polygon_distance(centers(:,:,disk)',scene.obstacles{obstacle}))-radius);end;end
score=8*max(0,.18-gap)+30*max(0,max(abs(phi))-.53);
end
