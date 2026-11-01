function candidate=seed_trajectory(task,nodes)
% Obstacle-aware Hermite routes with zero-end-slope lateral bumps.
count=size(task.start,2);candidate.state=zeros(6,nodes,count);candidate.control=zeros(2,nodes-1,count);
horizon=35;grid=linspace(0,1,nodes);progress=10*grid.^3-15*grid.^4+6*grid.^5;
for car=1:count
 begin=task.start(:,car);goal=task.goal(:,car);span=norm(goal(1:2)-begin(1:2));best=inf;
 for direction=[1,-1]
  for scale=[.7,1.3,2.0]
   for bend=[0,-1.0,1.0,-2.0,2.0]
   t=progress;u=scale*span*direction*[cos(begin(3));sin(begin(3))];v=scale*span*direction*[cos(goal(3));sin(goal(3))];
   xy=begin(1:2)*(2*t.^3-3*t.^2+1)+u*(t.^3-2*t.^2+t)+goal(1:2)*(-2*t.^3+3*t.^2)+v*(t.^3-t.^2);
   tangent=begin(1:2)*(6*t.^2-6*t)+u*(3*t.^2-4*t+1)+goal(1:2)*(-6*t.^2+6*t)+v*(3*t.^2-2*t);
   normal=[-(goal(2)-begin(2));goal(1)-begin(1)]/span;
   xy=xy+normal*(bend*16*t.^2.*(1-t).^2);
   tangent=tangent+normal*(bend*32*t.*(1-t).*(1-2*t));
   angle=unwrap(atan2(direction*tangent(2,:),direction*tangent(1,:)));angle=angle+2*pi*round((begin(3)-angle(1))/(2*pi));
   % Reject a different heading winding before applying endpoint equalities.
   if abs(angle(end)-goal(3))>1e-6,continue;end
   q=zeros(6,nodes);q(1:3,:)=[xy;angle];speed=direction*hypot(gradient(xy(1,:)),gradient(xy(2,:)))/(horizon/(nodes-1));q(4,:)=speed;
   q(5,:)=atan(task.geometry.LW*gradient(angle)./(sign(speed).*max(abs(speed),.08)*(horizon/(nodes-1))));
   q(5,abs(speed)<.08)=0;q(6,:)=gradient(speed)/(horizon/(nodes-1));
   [sq,rad]=collision_vector(reshape(q,6,nodes,1),task);
   score=1000*sum(max(0,rad+.3-sqrt(sq)).^2,'all')+sum(abs(diff(angle)))+sum(max(0,abs(q(5,:))-.7).^2)*10;
   if score<best,best=score;chosen=q;end
   end
  end
 end
 chosen(:,1)=[begin;0;0;0];chosen(:,end)=[goal;0;0;0];
 candidate.state(:,:,car)=chosen;
 candidate.control(1,:,car)=diff(chosen(6,:))/(horizon/(nodes-1));
 candidate.control(2,:,car)=diff(chosen(5,:))/(horizon/(nodes-1));
end
candidate.horizon=horizon;
end
