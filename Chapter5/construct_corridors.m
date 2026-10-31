function corridor=construct_corridors(scene,reference)
count=size(reference,3);steps=scene.nodes;g=scene.geometry;radius=hypot(g.LW+g.LF+g.LR,g.LB)/2;
offset=(g.LW+g.LF-g.LR)/2;
for car=1:count,reference(9:10,:,car)=reference(1:2,:,car)+offset*[cos(reference(3,:,car));sin(reference(3,:,car))];end
corridor.absolute=zeros(4,steps,count);corridor.pairs=nchoosek(1:count,2);corridor.relative=zeros(4,steps,size(corridor.pairs,1));corridor.repaired_absolute=0;corridor.repaired_relative=0;
for car=1:count
 for k=1:steps
  seed=reference(9:10,k,car);test=@(box)absolute_free(box,scene,radius+scene.margin);
  if ~test(seed([1,1,2,2])'),seed=repair(seed,test);corridor.repaired_absolute=corridor.repaired_absolute+1;end
  corridor.absolute(:,k,car)=expand(seed,test,.5,4);
 end
end
keep=true(size(corridor.pairs,1),1);
for pair=1:size(corridor.pairs,1)
 cars=corridor.pairs(pair,:);a=corridor.absolute(:,:,cars(1));b=corridor.absolute(:,:,cars(2));
 gap=hypot(max(max(a(1,:)-b(2,:),b(1,:)-a(2,:)),0),max(max(a(3,:)-b(4,:),b(3,:)-a(4,:)),0));
 if all(gap>=2*radius+scene.margin),keep(pair)=false;continue;end
 limit=2*radius+scene.margin;test=@(box)hypot(max([box(1),-box(2),0]),max([box(3),-box(4),0]))>=limit;
 for k=1:steps
  seed=reference(9:10,k,cars(1))-reference(9:10,k,cars(2));
  if norm(seed)<limit+.02
   if norm(seed)<.01,seed=[1;.2];end
   seed=seed/norm(seed)*(limit+.03);corridor.repaired_relative=corridor.repaired_relative+1;
  end
  corridor.relative(:,k,pair)=expand(seed,test,.5,5);
 end
end
corridor.pairs=corridor.pairs(keep,:);corridor.relative=corridor.relative(:,:,keep);
end
function safe=absolute_free(box,scene,radius)
b=scene.bounds;safe=box(1)>=b(1)+radius&&box(2)<=b(2)-radius&&box(3)>=b(3)+radius&&box(4)<=b(4)-radius;
if ~safe,return;end
for index=1:numel(scene.obstacles)
 if polygon_box_distance(box,scene.obstacles{index})<radius,safe=false;return;end
end
end
function seed=repair(seed,test)
for radius=.25:.25:12
 for angle=linspace(0,2*pi,25)
  trial=seed+radius*[cos(angle);sin(angle)];
  if test(trial([1,1,2,2])'),seed=trial;return;end
 end
end
error('No valid corridor seed within repair budget.');
end
function box=expand(seed,test,step,limit)
box=seed([1,1,2,2])';active=true(1,4);growth=zeros(1,4);
while any(active)
 for side=[1,4,2,3]
  if ~active(side),continue;end
  candidate=box;candidate(side)=candidate(side)+step*(1-2*ismember(side,[1,3]));
  if growth(side)+step<=limit && test(candidate)
   box=candidate;growth(side)=growth(side)+step;
  else
   for fine=step./[2,4,8,16]
    candidate=box;candidate(side)=candidate(side)+fine*(1-2*ismember(side,[1,3]));
    if growth(side)+fine<=limit && test(candidate),box=candidate;growth(side)=growth(side)+fine;end
   end
   active(side)=false;
  end
 end
end
box=box';
end
