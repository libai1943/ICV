function centers=disc_centers(state,scene)
[~,offsets]=disc_geometry(scene);centers=zeros(2,size(state,2),numel(offsets),size(state,3));
for car=1:size(state,3)
 for d=1:numel(offsets),centers(:,:,d,car)=state(1:2,:,car)+offsets(d)*[cos(state(3,:,car));sin(state(3,:,car))];end
end
end
