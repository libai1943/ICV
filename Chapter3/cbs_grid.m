function [paths,report]=cbs_grid(blocked,starts,goals,horizon)
% Conflict-based search, minimum sum of arrival times in the bounded graph.
% Exact low-level search; all agents remain at their goals after arrival.
N=size(starts,1);empty=struct('vertex',zeros(0,3),'edge',zeros(0,5));root.constraints=repmat(empty,1,N);root.paths=cell(1,N);root.cost=0;
for i=1:N
 [root.paths{i},c]=space_time_astar(blocked,starts(i,:),goals(i,:),empty,horizon);
 assert(isfinite(c),'Individual task infeasible within the horizon.');root.cost=root.cost+c;
end
open={root};expanded=0;
while ~isempty(open)
 [~,ix]=min(cellfun(@(n)n.cost,open));node=open{ix};open(ix)=[];expanded=expanded+1;
 assert(expanded<=10000,'CBS search limit reached; no completeness claim beyond this limit.');
 conflict=first_conflict(node.paths);
 if isempty(conflict),paths=node.paths;report=struct('sum_of_costs',node.cost,'expanded_nodes',expanded,'horizon',horizon);return;end
 for id=[conflict.i,conflict.j]
  child=node;
  if strcmp(conflict.type,'vertex')
   child.constraints(id).vertex(end+1,:)=[conflict.a,conflict.time];
  else
   if id==conflict.i,e=[conflict.a,conflict.a1,conflict.time];else,e=[conflict.b,conflict.b1,conflict.time];end
   child.constraints(id).edge(end+1,:)=e;
  end
  [p,c]=space_time_astar(blocked,starts(id,:),goals(id,:),child.constraints(id),horizon);
  if ~isfinite(c),continue;end
  child.paths{id}=p;child.cost=sum(cellfun(@(q)size(q,1)-1,child.paths));open{end+1}=child; %#ok<AGROW>
 end
end
error('No conflict-free solution within the finite time horizon.');
end
