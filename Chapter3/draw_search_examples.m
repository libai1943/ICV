function report=draw_search_examples(out)
%DRAW_SEARCH_EXAMPLES Figures from the accompanying search algorithms.
set(groot,'defaultAxesFontName','Times New Roman','defaultTextFontName','Times New Roman','defaultTextInterpreter','tex');
blue=[.10,.36,.72];gold=[.89,.56,.10];map=false(9,12);map(2:7,6)=true;map(5,6)=false;map(7,8:10)=true;start=[2,2];goal=[8,11];
f=figure('Color','w','Position',[50,50,1100,480],'Visible','off');
for panel=1:2
 ax=axes(f,'Position',[.04+(panel-1)*.5,.16,.43,.80]);hold(ax,'on');gridmap(ax,map);conn=4*panel;[p,c,e]=astar_grid(map,start,goal,conn,true);
 plot(ax,p(:,2),p(:,1),'-o','Color',blue,'LineWidth',1.7,'MarkerSize',4,'MarkerFaceColor','w');
 text(ax,start(2),start(1)-.6,'S','HorizontalAlignment','center','FontSize',20);text(ax,goal(2),goal(1)+.6,'G','HorizontalAlignment','center','FontSize',20);
 subcap(ax,sprintf('(%c) %d-connected: cost %.5g',96+panel,conn,c));report.(['astar',num2str(conn)])=struct('cost',c,'expanded_nodes',e,'path',p);
end
savefigure(f,out,'fig03-03');
map=true(3,5);map(2,:)=false;C=struct('vertex',[2,3,2;2,3,3],'edge',zeros(0,5));
[p,c]=space_time_astar(map,[2,1],[2,5],C,10);assert(c==6);assert(sum(all(diff(p)==0,2))==2);
report.space_time=struct('path',p,'arrival_time',c,'waits',2,'forbidden_vertices',C.vertex);
f=figure('Color','w','Position',[50,50,1100,480],'Visible','off');
ax=axes(f,'Position',[.04,.22,.43,.70]);hold(ax,'on');gridmap(ax,map);
plot(ax,[1,5],[2,2],'-o','Color',blue,'LineWidth',1.6,'MarkerSize',5);patch(ax,[2.5,3.5,3.5,2.5],[1.5,1.5,2.5,2.5],[.97,.88,.71],'FaceAlpha',.65,'EdgeColor',gold,'LineWidth',1.1);
text(ax,3,3.65,'Cell (2,3) unavailable at k = 2, 3','HorizontalAlignment','center','FontSize',18);
text(ax,1,1.1,'S','HorizontalAlignment','center','FontSize',20);text(ax,5,1.1,'G','HorizontalAlignment','center','FontSize',20);annotation(f,'textbox',[.04,.02,.43,.07],'String','(a) Spatial corridor','EdgeColor','none','HorizontalAlignment','center','FontName','Times New Roman','FontSize',21);
ax=axes(f,'Position',[.59,.22,.35,.70]);hold(ax,'on');for k=0:6,plot(ax,1:5,k*ones(1,5),'.','Color',[.75,.75,.75],'MarkerSize',10);end
plot(ax,p(:,2),0:c,'-o','Color',blue,'LineWidth',1.8,'MarkerFaceColor','w','MarkerSize',6);plot(ax,[3,3],[2,3],'x','Color',gold,'LineWidth',2,'MarkerSize',12);
xlim(ax,[.5,5.5]);ylim(ax,[-.3,6.4]);xticks(ax,1:5);yticks(ax,0:6);xlabel(ax,'Column');ylabel(ax,'Time index k');box(ax,'off');set(ax,'FontSize',17);
annotation(f,'textbox',[.54,.02,.43,.07],'String','(b) Space-time slice at row 2','EdgeColor','none','HorizontalAlignment','center','FontName','Times New Roman','FontSize',21);savefigure(f,out,'fig03-04');
empty=struct('vertex',zeros(0,3),'edge',zeros(0,5));map=false(3);starts=[2,1;2,3];goals=[2,3;2,1];root=cell(1,2);
for i=1:2,[root{i},~]=space_time_astar(map,starts(i,:),goals(i,:),empty,10);end
cf=first_conflict(root);assert(strcmp(cf.type,'vertex')&&cf.time==1);childcost=zeros(1,2);
for i=1:2
 C=empty;C.vertex=[cf.a,cf.time];[pc,~]=space_time_astar(map,starts(i,:),goals(i,:),C,10);child=root;child{i}=pc;childcost(i)=sum(cellfun(@(q)size(q,1)-1,child));cc=first_conflict(child);assert(strcmp(cc.type,'edge'));
end
[paths,stats]=cbs_grid(map,starts,goals,10);assert(isempty(first_conflict(paths)));assert(stats.sum_of_costs==6);
report.cbs=stats;report.cbs.paths=paths;report.cbs.first_child_costs=childcost;
f=figure('Color','w','Position',[50,50,1100,910],'Visible','off');cols=[blue;gold];
for panel=[1,3]
 ax=axes(f,'Position',[.06,.59-(panel==3)*.47,.37,.35]);hold(ax,'on');gridmap(ax,map);if panel==1,pp=root;else,pp=paths;end
 for i=1:2
  off=(i-1.5)*.12;plot(ax,pp{i}(:,2),pp{i}(:,1)+off,'-o','Color',cols(i,:),'LineWidth',1.7,'MarkerSize',6,'MarkerFaceColor','w');
  for k=1:size(pp{i},1),tx=pp{i}(k,2);ty=pp{i}(k,1)+off+(i-1.5)*.42;if panel==3&&i==1&&k==1,tx=tx-.22;ty=pp{i}(k,1)+off;end;text(ax,tx,ty,num2str(k-1),'FontSize',16,'Color',cols(i,:),'HorizontalAlignment','center');end
 end
 if panel==1,plot(ax,2,2,'kx','MarkerSize',22,'LineWidth',1.4);subcap(ax,'(a) Root: conflict at k = 1');else,subcap(ax,'(c) Conflict-free paths; cost 6');end
end
ax=axes(f,'Position',[.53,.59,.43,.35]);hold(ax,'on');axis(ax,[0,10,0,7]);axis(ax,'off');
node(ax,[5,5.7],{'Root: cost 4','Both use (2,2) at k = 1'});plot(ax,[5,2.4],[4.9,3.2],'k-');plot(ax,[5,7.6],[4.9,3.2],'k-');
node(ax,[2.4,2.4],{'Forbid agent 1','at (2,2), k = 1','Cost 5; edge conflict'});node(ax,[7.6,2.4],{'Forbid agent 2','at (2,2), k = 1','Cost 5; edge conflict'});subcap(ax,'(b) First high-level split');
ax=axes(f,'Position',[.53,.12,.43,.35]);hold(ax,'on');axis(ax,[0,7,0,4]);axis(ax,'off');H=max(cellfun(@(q)size(q,1),paths))-1;
for k=0:H
 text(ax,1.9+k,3.4,num2str(k),'FontSize',19,'HorizontalAlignment','center');
 for i=1:2,q=paths{i}(min(k+1,size(paths{i},1)),:);text(ax,1.9+k,3.05-i,sprintf('(%d,%d)',q(1),q(2)),'FontSize',18,'Color',cols(i,:),'HorizontalAlignment','center');end
end
text(ax,.4,3.4,'k','FontSize',19);text(ax,0,2.05,'Agent 1','FontSize',18);text(ax,0,1.05,'Agent 2','FontSize',18);plot(ax,[0,6.7],[2.9,2.9],'k-');subcap(ax,'(d) Occupancy (row, column)');
savefigure(f,out,'fig03-05');fid=fopen(fullfile(out,'illustrated_examples.json'),'w');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
function gridmap(ax,map)
[nr,nc]=size(map);for r=1:nr,for c=1:nc,col=[1,1,1];if map(r,c),col=[.72,.72,.72];end;patch(ax,c+[-.5,.5,.5,-.5],r+[-.5,-.5,.5,.5],col,'EdgeColor',[.82,.82,.82],'LineWidth',.6);end,end
axis(ax,'equal');axis(ax,[.45,nc+.55,.45,nr+.55]);axis(ax,'off');
end
function subcap(ax,s)
text(ax,.5,-.13,s,'Units','normalized','HorizontalAlignment','center','VerticalAlignment','top','FontName','Times New Roman','FontSize',21,'Interpreter','none');
end
function node(ax,c,lines)
rectangle(ax,'Position',[c(1)-2.25,c(2)-.85,4.5,1.7],'FaceColor',[.95,.96,.97],'EdgeColor',[.4,.4,.4]);text(ax,c(1),c(2),lines,'HorizontalAlignment','center','VerticalAlignment','middle','FontSize',16);
end
function savefigure(f,out,n)
exportgraphics(f,fullfile(out,[n,'.pdf']),'ContentType','vector');exportgraphics(f,fullfile(out,[n,'.png']),'Resolution',220);close(f);
end
