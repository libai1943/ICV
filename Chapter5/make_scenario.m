function scene=make_scenario(count)
if nargin<1,count=8;end
scene.geometry=struct('LW',2.8,'LF',.96,'LR',.929,'LB',1.942);
scene.nodes=161;scene.horizon=80;scene.margin=.25;scene.maxSpeed=5;scene.maxAcceleration=1.2;scene.maxJerk=1;scene.maxSteering=.6;scene.maxSteeringRate=.4;
scene.bounds=[-65,65,-65,65];
% Four clipped industrial blocks bound a four-arm intersection without lanes.
scene.obstacles={[-65,6;-11,6;-7.8,10;-7.8,65;-65,65], ...
 [7,65;7,11;11,6.2;65,6.2;65,65], ...
 [65,-6.4;10,-6.4;7.4,-11;7.4,-65;65,-65], ...
 [-65,-65;-7,-65;-7,-10;-11,-6;-65,-6]};
scene.start=zeros(6,count);scene.goal=zeros(6,count);scene.routes=cell(1,count);
outCount=zeros(1,4);headings=[0,pi/2,pi,-pi/2];turns=[1,-1,1,-1,0,0,0,0,-1,1,-1,1];
for car=1:count
 approach=mod(car-1,4)+1;queue=floor((car-1)/4);angle=headings(approach);turn=turns(mod(car-1,numel(turns))+1);exitAngle=angle+turn*pi/2;
 destination=mod(approach-1+turn,4)+1;rank=outCount(destination);outCount(destination)=rank+1;
 forward=[cos(angle);sin(angle)];right=[sin(angle);-cos(angle)];exitForward=[cos(exitAngle);sin(exitAngle)];exitRight=[sin(exitAngle);-cos(exitAngle)];
 begin=-(25+7*queue)*forward+2.9*right;goal=(40-7*rank)*exitForward+2.9*exitRight;
 scene.start(:,car)=[begin;angle;0;0;0];scene.goal(:,car)=[goal;exitAngle;0;0;0];
 lane=2.9;startDistance=25+7*queue;goalDistance=40-7*rank;
 if turn==0
  local=[linspace(-startDistance,goalDistance,501);-lane*ones(1,501)];
 elseif turn==1
  radius=6+lane;angles=linspace(-pi/2,0,201);arc=[-6+radius*cos(angles);6+radius*sin(angles)];
  local=[[-startDistance+(startDistance-6)*linspace(0,1,151);-lane*ones(1,151)],arc(:,2:end),[lane*ones(1,151);6+(goalDistance-6)*linspace(0,1,151)]];
 else
  radius=8-lane;angles=linspace(pi/2,0,201);arc=[-8+radius*cos(angles);-8+radius*sin(angles)];
  local=[[-startDistance+(startDistance-8)*linspace(0,1,151);-lane*ones(1,151)],arc(:,2:end),[-lane*ones(1,151);-8-(goalDistance-8)*linspace(0,1,151)]];
 end
 rotation=[cos(angle),-sin(angle);sin(angle),cos(angle)];path=rotation*local;
 path=path(:,[true,vecnorm(diff(path,1,2))>1e-7]);
 scene.routes{car}=path;
end
scene.id=sprintf('intersection_%02d',count);
end
