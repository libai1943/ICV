function [path,cost,expanded]=astar_grid(blocked,start,goal,connectivity,useHeuristic)
% Grid point-agent A*: row/column coordinates; unit cardinal edge costs.
% Eight-connected diagonals cost sqrt(2) and cannot cut blocked corners.
if nargin<5,useHeuristic=true;end
[nr,nc]=size(blocked);g=inf(nr,nc);parent=zeros(nr,nc);closed=false(nr,nc);
assert(~blocked(start(1),start(2))&&~blocked(goal(1),goal(2)));
s=sub2ind([nr,nc],start(1),start(2));target=sub2ind([nr,nc],goal(1),goal(2));g(s)=0;
open=[s,heuristic(start),0];expanded=0;moves=[-1,0;1,0;0,-1;0,1];
if connectivity==8,moves=[moves;-1,-1;-1,1;1,-1;1,1];else,assert(connectivity==4);end
while ~isempty(open)
 [~,ix]=min(open(:,2));entry=open(ix,:);open(ix,:)=[];id=entry(1);
 if entry(3)>g(id)+1e-12,continue;end
 if closed(id),continue;end
 closed(id)=true;expanded=expanded+1;[r,c]=ind2sub([nr,nc],id);
 if id==target
  ids=id;while ids(1)~=s,ids=[parent(ids(1)),ids];end %#ok<AGROW>
  [rr,cc]=ind2sub([nr,nc],ids);path=[rr(:),cc(:)];cost=g(id);return;
 end
 for j=1:size(moves,1)
  q=[r,c]+moves(j,:);if any(q<1)||q(1)>nr||q(2)>nc||blocked(q(1),q(2)),continue;end
  if all(moves(j,:)~=0)&&(blocked(r,q(2))||blocked(q(1),c)),continue;end
  next=sub2ind([nr,nc],q(1),q(2));trial=g(id)+norm(moves(j,:));
  if trial<g(next)-1e-12
   g(next)=trial;parent(next)=id;closed(next)=false;open(end+1,:)=[next,trial+heuristic(q),trial]; %#ok<AGROW>
  end
 end
end
path=[];cost=inf;
 function h=heuristic(q)
  delta=abs(q-goal);if connectivity==8,h=max(delta)+(sqrt(2)-1)*min(delta);else,h=sum(delta);end
  if ~useHeuristic,h=0;end
 end
end
