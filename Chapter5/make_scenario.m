function scene=make_scenario(count)
% Five unequal, curved approaches and a flattened, lane-free meeting area.
% The seed controls geometry, not a solver's success/failure classification.
if nargin<1,count=12;end
rng(170+count,'twister');
scene.geometry=struct('LW',2.8,'LF',.96,'LR',.929,'LB',1.942);
scene.nodes=241;scene.horizon=120;scene.margin=.25;scene.maxSpeed=5;scene.maxAcceleration=1.2;scene.maxJerk=1;scene.maxSteering=.6;scene.maxSteeringRate=.4;
scene.bounds=[-90,90,-90,90];scene.seed=170+count;
angles=([8,65,147,213,286]+3*(rand(1,5)-.5))*pi/180;
widths=[7.0,7.6,7.2,8.0,6.8];bends=[6,-7,5,-6,7]+rand(1,5)-.5;
arms=cell(1,5);left=arms;right=arms;inner=zeros(2,5);tangent=inner;
for j=1:5
 direction=[cos(angles(j));sin(angles(j))];normal=[-direction(2);direction(1)];
 inner(:,j)=[14*cos(angles(j));9*sin(angles(j))];
 s=linspace(0,100,401);shift=bends(j)*(1-exp(-(s/24).^2));
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
 scene.obstacles{j}=[left{j}(:,1:16:end),exterior,fliplr(right{next}(:,1:16:end))]';
end
scene.arms=arms;scene.approach_widths=widths;scene.start=zeros(6,count);scene.goal=zeros(6,count);scene.routes=cell(1,count);
entries=[1,3,5];exits=[4,2,2,2,4,4];outCount=zeros(1,5);
for car=1:count
 a=entries(mod(car-1,3)+1);b=exits(mod(car-1,6)+1);queue=floor((car-1)/3);
 rank=outCount(b);outCount(b)=rank+1;
 inbound=arms{a};outbound=arms{b};ain=[0,cumsum(vecnorm(diff(inbound,1,2)))];aout=[0,cumsum(vecnorm(diff(outbound,1,2)))];
 first=interp1(ain,inbound',linspace(27+6.5*queue,0,201),'pchip')';
 last=interp1(aout,outbound',linspace(0,65-6.5*rank,301),'pchip')';
 t=linspace(0,1,241);control=[inner(:,a),inner(:,a)-13*tangent(:,a),inner(:,b)-13*tangent(:,b),inner(:,b)];
 connector=control*[ (1-t).^3;3*(1-t).^2.*t;3*(1-t).*t.^2;t.^3 ];
 route=[first,connector(:,2:end),last(:,2:end)];route=route(:,[true,vecnorm(diff(route,1,2))>1e-8]);
 theta=unwrap(atan2(gradient(route(2,:)),gradient(route(1,:))));
 scene.start(:,car)=[route(:,1);theta(1);0;0;0];scene.goal(:,car)=[route(:,end);theta(end);0;0;0];scene.routes{car}=route;
end
scene.id=sprintf('irregular_five_arm_%02d',count);
end
