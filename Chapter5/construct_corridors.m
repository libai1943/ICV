function corridor=construct_corridors(scene,reference)
[radius,offsets]=disc_geometry(scene);nd=numel(offsets);count=size(reference,3);steps=scene.nodes;
centers=disc_centers(reference,scene);corridor.absolute=zeros(4,steps,nd,count);corridor.repaired_absolute=0;corridor.repaired_relative=0;
for car=1:count
 for disk=1:nd
  for k=1:steps
   seed=centers(:,k,disk,car);test=@(box)absolute_free(box,scene,radius+scene.margin);
   if ~test(seed([1,1,2,2])'),seed=repair(seed,test);corridor.repaired_absolute=corridor.repaired_absolute+1;end
   corridor.absolute(:,k,disk,car)=expand(seed,test,.25,1.2);
  end
 end
end
% Screen each disc pair at each time using the complete absolute boxes.
% Only nonscreened pair-time tuples need relative variables and constraints.
corridor.pairs=zeros(0,5);corridor.relative=zeros(4,0);limit=2*radius+scene.margin;
test=@(box)hypot(max([box(1),-box(2),0]),max([box(3),-box(4),0]))>=limit;
for car=1:count
 for other=car+1:count
  for da=1:nd
   for db=1:nd
    a=corridor.absolute(:,:,da,car);b=corridor.absolute(:,:,db,other);
    gap=hypot(max(max(a(1,:)-b(2,:),b(1,:)-a(2,:)),0),max(max(a(3,:)-b(4,:),b(3,:)-a(4,:)),0));
    active=find(gap<limit);
    for k=active
     seed=centers(:,k,da,car)-centers(:,k,db,other);
     if norm(seed)<limit+.01
      if norm(seed)<.01,seed=[1;.2];end
      seed=seed/norm(seed)*(limit+.02);corridor.repaired_relative=corridor.repaired_relative+1;
     end
     corridor.pairs(end+1,:)=[car,da,other,db,k];corridor.relative(:,end+1)=expand(seed,test,.25,2);
    end
   end
  end
 end
end
% Certified separation of the pose-linear interpolant over whole intervals.
% Heading-increment trust bounds give a fixed chord-deviation allowance.
maxYaw=scene.maxSpeed*tan(scene.maxSteering)/scene.geometry.LW*scene.horizon/(steps-1);
corridor.yaw_bound=min(maxYaw,abs(diff(reference(3,:,:),1,2))+.10);
corridor.guards=zeros(0,5);corridor.guard_normal=zeros(2,0);corridor.guard_limit=zeros(1,0);
for car=1:count
 for other=car+1:count
  for da=1:nd
   for db=1:nd
    a=corridor.absolute(:,:,da,car);b=corridor.absolute(:,:,db,other);
    aa=[min(a(1,1:end-1),a(1,2:end));max(a(2,1:end-1),a(2,2:end));min(a(3,1:end-1),a(3,2:end));max(a(4,1:end-1),a(4,2:end))];
    bb=[min(b(1,1:end-1),b(1,2:end));max(b(2,1:end-1),b(2,2:end));min(b(3,1:end-1),b(3,2:end));max(b(4,1:end-1),b(4,2:end))];
    padding=abs(offsets(da))*corridor.yaw_bound(1,:,car).^2/8+abs(offsets(db))*corridor.yaw_bound(1,:,other).^2/8;
    limit=2*radius+scene.margin+padding;
    gap=hypot(max(max(aa(1,:)-bb(2,:),bb(1,:)-aa(2,:)),0),max(max(aa(3,:)-bb(4,:),bb(3,:)-aa(4,:)),0));
    for k=find(gap<limit)
     r0=centers(:,k,da,car)-centers(:,k,db,other);r1=centers(:,k+1,da,car)-centers(:,k+1,db,other);dr=r1-r0;
     alpha=max(0,min(1,-dot(r0,dr)/max(dot(dr,dr),1e-12)));nearest=r0+alpha*dr;
     if norm(nearest)<1e-8,nearest=[-dr(2);dr(1)];end
     if norm(nearest)<1e-8,nearest=[1;0];end
     normal=nearest/norm(nearest);
     corridor.guards(end+1,:)=[car,da,other,db,k];corridor.guard_normal(:,end+1)=normal;corridor.guard_limit(end+1)=limit(k);
    end
   end
  end
 end
end

end
function safe=absolute_free(box,scene,radius)
b=scene.bounds;safe=box(1)>=b(1)+radius&&box(2)<=b(2)-radius&&box(3)>=b(3)+radius&&box(4)<=b(4)-radius;
if ~safe,return;end
for index=1:numel(scene.obstacles),if polygon_box_distance(box,scene.obstacles{index})<radius,safe=false;return;end;end
end
function seed=repair(seed,test)
for radius=.1:.1:3
 for angle=linspace(0,2*pi,33)
  trial=seed+radius*[cos(angle);sin(angle)];if test(trial([1,1,2,2])'),seed=trial;return;end
 end
end
error('No valid corridor seed within repair budget.');
end
function box=expand(seed,test,step,limit)
box=seed([1,1,2,2])';half=.04;
while ~test(box+half*[-1,1,-1,1])&&half>1e-7,half=half/2;end
if test(box+half*[-1,1,-1,1]),box=box+half*[-1,1,-1,1];end
active=true(1,4);growth=half*ones(1,4);
while any(active)
 for side=[1,4,2,3]
  if ~active(side),continue;end
  candidate=box;candidate(side)=candidate(side)+step*(1-2*ismember(side,[1,3]));
  if growth(side)+step<=limit&&test(candidate),box=candidate;growth(side)=growth(side)+step;
  else
   for fine=step./[2,4,8,16,32,64]
    candidate=box;candidate(side)=candidate(side)+fine*(1-2*ismember(side,[1,3]));
    if growth(side)+fine<=limit&&test(candidate),box=candidate;growth(side)=growth(side)+fine;end
   end
   active(side)=false;
  end
 end
end
box=box';
end
