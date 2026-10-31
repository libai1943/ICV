function report=validate_motion(task,q)
dt=task.dt;count=size(q,3);N=size(q,2);sub=20;dense=zeros(3,(N-1)*sub+1,count);defect=0;speed=0;
for i=1:count
 rhs=[q(4,1:end-1,i).*cos(q(3,1:end-1,i));q(4,1:end-1,i).*sin(q(3,1:end-1,i));q(4,1:end-1,i).*tan(q(5,1:end-1,i))/task.geometry.LW;q(6:7,1:end-1,i)];
 defect=max(defect,max(abs(q(1:5,2:end,i)-q(1:5,1:end-1,i)-dt*rhs),[],'all'));
 for k=1:N-1,at=(k-1)*sub+(1:sub);dense(:,at,i)=q(1:3,k,i)+(q(1:3,k+1,i)-q(1:3,k,i))*(0:sub-1)/sub;end
 dense(:,end,i)=q(1:3,end,i);rate=diff(q(1:3,:,i),1,2)/dt;speed=max(speed,max(vecnorm(rate(1:2,:))+task.offset*abs(rate(3,:))));
end
centers=dense(1:2,:,:)+task.offset*cat(1,cos(dense(3,:,:)),sin(dense(3,:,:)));clearance=inf(1,size(dense,2));
for i=1:count-1,for j=i+1:count,clearance=min(clearance,vecnorm(centers(:,:,i)-centers(:,:,j))-2*task.radius);end,end
bound=min(clearance)-speed*dt/sub;
violation=max([0;reshape(abs(q(4,:,:))-task.maxSpeed,[],1);reshape(abs(q(5,:,:))-task.maxSteering,[],1);reshape(abs(q(6,:,:))-task.maxAcceleration,[],1);reshape(abs(q(7,:,:))-task.maxSteeringRate,[],1)]);
report=struct('maximum_euler_defect',defect,'bound_violation',violation,'minimum_sampled_clearance',min(clearance),'clearance_bound',bound,'interpolation','piecewise-linear rear-axle position and unwrapped heading','passed',defect<1e-5&&violation<1e-6&&bound>=0);
end
