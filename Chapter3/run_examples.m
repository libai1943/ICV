function report=run_examples()
root=fileparts(mfilename('fullpath'));addpath(root);out=fullfile(root,'output');if ~isfolder(out),mkdir(out);end
map=false(9,12);map(2:7,6)=true;map(5,6)=false;map(7,8:10)=true;
for conn=[4,8]
 [p,c,e]=astar_grid(map,[2,2],[8,11],conn,true);[~,d,~]=astar_grid(map,[2,2],[8,11],conn,false);assert(abs(c-d)<1e-10);
 report.(['astar',num2str(conn)])=struct('cost',c,'expanded_nodes',e,'path',p);
end
% Diagonal corner-cutting is prohibited.
m=[false,true;true,false];[p,c]=astar_grid(m,[1,1],[2,2],8,true);assert(isempty(p)&&isinf(c));
% A future goal restriction must prevent early permanent occupancy.
C=struct('vertex',[2,3,4],'edge',zeros(0,5));[p,c]=space_time_astar(false(3),[2,1],[2,3],C,10);assert(c>4);
% A swap cannot be resolved by checking vertices alone.
p1=[2,1;2,2];p2=[2,2;2,1];cf=first_conflict({p1,p2});assert(strcmp(cf.type,'edge'));
[paths,cbs]=cbs_grid(false(3),[2,1;2,3],[2,3;2,1],10);assert(isempty(first_conflict(paths)));assert(cbs.sum_of_costs==6);
% Independent exhaustive check of all candidate pairs with total cost < 6.
A=shortPaths([2,1],[2,3]);B=shortPaths([2,3],[2,1]);
for i=1:numel(A)
 for j=1:numel(B)
  if size(A{i},1)+size(B{j},1)-2<6,assert(~isempty(first_conflict({A{i},B{j}})));end
 end
end
report.cbs=cbs;report.cbs.paths=paths;report.gradient=gradient_example(out);report.passed=true;
draw_connectivity(out);report.illustrated=draw_search_examples(out);fid=fopen(fullfile(out,'validation.json'),'w');fprintf(fid,'%s',jsonencode(report));fclose(fid);disp(report);
end

function paths=shortPaths(start,goal)
paths={};front={start};moves=[-1,0;1,0;0,-1;0,1;0,0];
for depth=0:3
 next={};
 for j=1:numel(front)
  p=front{j};if isequal(p(end,:),goal),paths{end+1}=p;end
  if depth==3,continue;end
  for k=1:5
   q=p(end,:)+moves(k,:);if all(q>=1)&&all(q<=3),next{end+1}=[p;q];end
  end
 end
 front=next;
end
end
