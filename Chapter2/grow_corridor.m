function [box,history]=grow_corridor(seed,obstacles,r,bounds,step)
% Cyclic axis-aligned expansion with an exact original-polygon distance test.
% A box for a disc center is accepted only if its full area stays farther
% than r from every obstacle. bounds=[xmin xmax ymin ymax].
box=[seed(1),seed(1),seed(2),seed(2)];active=true(1,4);history=[];
assert(free(box),'Corridor seed is not collision-free.');
while any(active)
 for side=[1,4,2,3]
  if ~active(side),continue;end
  trial=box;trial(side)=trial(side)+step*(1-2*ismember(side,[1,3]));ok=free(trial);
  history(end+1,:)=[side,ok,trial]; %#ok<AGROW>
  if ok,box=trial;else,active(side)=false;end
 end
end
 function ok=free(b)
  ok=b(1)>=bounds(1)&&b(2)<=bounds(2)&&b(3)>=bounds(3)&&b(4)<=bounds(4);
  if ~ok,return;end
  % Nondegenerate tiny rectangle also covers the point seed conservatively.
  epsbox=1e-9;V=[b(1)-epsbox,b(3)-epsbox;b(2)+epsbox,b(3)-epsbox;b(2)+epsbox,b(4)+epsbox;b(1)-epsbox,b(4)+epsbox];
  for z=1:numel(obstacles)
   if polygon_distance(V,obstacles{z})<=r+1e-9,ok=false;return;end
  end
 end
end
