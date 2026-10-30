function [path,cost]=space_time_astar(blocked,start,goal,C,horizon)
% Four-connected grid, unit-duration moves and waits, persistent goal occupancy.
% C.vertex rows: [row column time]. C.edge: [r0 c0 r1 c1 departureTime].
% The horizon bounds the graph; failure means no path within that horizon.
[nr,nc]=size(blocked);dims=[nr,nc,horizon+1];g=inf(dims);parent=zeros(dims);closed=false(dims);
path=[];cost=inf;
if blocked(start(1),start(2))||blocked(goal(1),goal(2))||forbiddenVertex(start,0),return;end
s=sub2ind(dims,start(1),start(2),1);g(s)=0;open=[s,sum(abs(start-goal)),0];moves=[-1,0;1,0;0,-1;0,1;0,0];
while ~isempty(open)
 [~,ix]=min(open(:,2));entry=open(ix,:);open(ix,:)=[];id=entry(1);
 if closed(id)||entry(3)>g(id),continue;end
 closed(id)=true;[r,c,k]=ind2sub(dims,id);t=k-1;
 if isequal([r,c],goal)&&goalClear(t)
  ids=id;while ids(1)~=s,ids=[parent(ids(1)),ids];end %#ok<AGROW>
  [rr,cc,~]=ind2sub(dims,ids);path=[rr(:),cc(:)];cost=t;return;
 end
 if t>=horizon,continue;end
 for j=1:5
  q=[r,c]+moves(j,:);if any(q<1)||q(1)>nr||q(2)>nc||blocked(q(1),q(2)),continue;end
  if forbiddenVertex(q,t+1),continue;end
  if ~isempty(C.edge)&&any(all(C.edge==[r,c,q,t],2)),continue;end
  next=sub2ind(dims,q(1),q(2),k+1);trial=t+1;
  if trial<g(next)
   g(next)=trial;parent(next)=id;open(end+1,:)=[next,trial+sum(abs(q-goal)),trial]; %#ok<AGROW>
  end
 end
end
 function yes=forbiddenVertex(q,t)
  yes=~isempty(C.vertex)&&any(all(C.vertex==[q,t],2));
 end
 function yes=goalClear(t)
  yes=true;
  if ~isempty(C.vertex),yes=~any(all(C.vertex(:,1:2)==goal,2)&C.vertex(:,3)>=t);end
  if ~isempty(C.edge),yes=yes&&~any(all(C.edge(:,1:4)==[goal,goal],2)&C.edge(:,5)>=t);end
 end
end
