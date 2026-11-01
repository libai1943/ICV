function make_figures()
% Rebuild all Chapter4 illustrations and experimental plots from saved data.
root=fileparts(mfilename('fullpath'));addpath(fullfile(root,'common'));BookPlotStyle();colors=BookPalette();
out=fullfile(root,'output/figures');if ~isfolder(out),mkdir(out);end
s=load(fullfile(root,'output/baseline/book_result.mat'));book=s.book;p=book.parameters;g=book.geometry(1);
% 4.1: lane assignment and forward motion on a structured road.
f=figure('Color','w','Position',[50,50,1100,360],'Visible','off');ax=axes(f,'Position',[.085,.18,.89,.76]);hold(ax,'on');road(ax,[0,53],p);
for i=1:3
 x0=2+14*(i-1);y0=3.75*i;y1=3.75*(4-i);u=linspace(0,1,101);b=10*u.^3-15*u.^4+6*u.^5;
 x=x0+22*u;y=y0+(y1-y0)*b;plot(ax,x,y,'--','Color',colors(i,:),'LineWidth',1.2);
 k=23;theta=atan2((y1-y0)*(30*u(k)^2-60*u(k)^3+30*u(k)^4),22);
 BookDrawVehicle(ax,[x(k);y(k);theta;0;0],g,colors(i,:),i,14);
 plot(ax,x(end),y(end),'o','Color',colors(i,:),'MarkerFaceColor','w','MarkerSize',6);
end
ylabel(ax,'{\it y} (m)');xlabel(ax,'{\it x} (m)');yticks(ax,[3.75,7.5,11.25]);text(ax,51,p.road_upper-.45,'LB','HorizontalAlignment','right','FontSize',15);text(ax,51,p.road_lower+.45,'RB','HorizontalAlignment','right','FontSize',15);savefigs(f,out,'fig04-01');
% 4.2: two-disc body cover; the same footprint and orientation triangle.
f=figure('Color','w','Position',[50,50,900,450],'Visible','off');ax=axes(f,'Position',[.04,.07,.92,.88]);hold(ax,'on');axis(ax,'equal');axis(ax,[-2,6,-2.5,2.6]);axis(ax,'off');
L=g.LW+g.LF+g.LR;r=hypot(L/4,g.LB/2);q=[0;0;.09;0;0];e=[cos(q(3));sin(q(3))];cr=q(1:2)+(-g.LR+L/4)*e;cf=q(1:2)+(-g.LR+3*L/4)*e;
BookDrawVehicle(ax,q,g,colors(1,:),[],16);
for c={cr,cf},circle(ax,c{1},r,colors(2,:),1.2,'-');end

for c={cr,cf},plot(ax,c{1}(1),c{1}(2),'k.','MarkerSize',15);end
plot(ax,[cr(1),cr(1)],[cr(2),cr(2)+r],'k-','LineWidth',.9);text(ax,cr(1)+.18,cr(2)+r*.65,'{\rm R}_{\it i}','FontSize',20);
text(ax,cr(1),cr(2)-1.98,'{\bf c}_{\it i}^{\rm r}','HorizontalAlignment','center','FontSize',20);text(ax,cf(1),cf(2)-1.98,'{\bf c}_{\it i}^{\rm f}','HorizontalAlignment','center','FontSize',20);
ymax=2.25;plot(ax,[-2,6],[ymax,ymax],'k-','LineWidth',1);text(ax,5.8,ymax+.15,'LB','HorizontalAlignment','right','FontSize',18);
xx=cf(1)+r+.4;plot(ax,[xx,xx],[cf(2)+r,ymax],'k-','LineWidth',1);plot(ax,xx+[-.1,.1],[cf(2)+r,cf(2)+r],'k-');plot(ax,xx+[-.1,.1],[ymax,ymax],'k-');text(ax,xx+.2,(cf(2)+r+ymax)/2,'clearance','FontSize',16);savefigs(f,out,'fig04-02');
% Find a real conflict in the independently optimized maneuver collection.
blind=s.stage2;worst=inf;pair=[];kstar=1;
for i=1:p.N-1
 for j=i+1:p.N
  test=struct('time',blind.time,'state',blind.state(:,:,[i,j]),'geometry',book.geometry([i,j]));v=BookValidatePairs(test);[gap,k]=min(v.sampled_gap);
  if gap<worst,worst=gap;pair=[i,j];kstar=k;end
 end
end
assert(worst<0,'Expected an actual pre-translation conflict in this example.');
translated=blind;for i=1:p.N,translated.state(1,:,i)=translated.state(1,:,i)+book.translations(i);end
frames={blind,translated};offset=p.v0*blind.time(kstar);xx=[];
for a=frames,for i=pair,xx=[xx,squeeze(a{1}.state(1,:,i))-offset];end,end
xl=[5*floor(min(xx)/5)-3,5*ceil(max(xx)/5)+5];
f=figure('Color','w','Position',[50,50,1040,660],'Visible','off');
for panel=1:2
 ax=axes(f,'Position',[.07,.63-(panel-1)*.50,.90,.28]);hold(ax,'on');road(ax,xl,p);a=frames{panel};
 for i=pair
  plot(ax,squeeze(a.state(1,:,i))-offset,squeeze(a.state(2,:,i)),'--','Color',colors(i,:),'LineWidth',1.3);
  q=a.state(:,kstar,i);q(1)=q(1)-offset;BookDrawVehicle(ax,q,g,colors(i,:),i,14);
 end
 names={'(a) Conflict before longitudinal translation','(b) Same maneuver shapes after translation'};text(ax,.5,-.28,names{panel},'Units','normalized','HorizontalAlignment','center','FontSize',17.5);ylabel(ax,'{\it y} (m)');
end
savefigs(f,out,'fig04-03');
% 4.4: actual priority construction, original and translated formation.
[~,priority]=sort(book.task(:,1),'descend');xl=[5*floor(min(translated.state(1,1,:))/5)-4,8];
f=figure('Color','w','Position',[50,50,1050,550],'Visible','off');
for panel=1:2
 ax=axes(f,'Position',[.06,.57-(panel-1)*.42,.91,.32]);hold(ax,'on');road(ax,xl,p);a=frames{panel};
 for i=1:p.N,BookDrawVehicle(ax,a.state(:,1,i),g,colors(i,:),i,13);end
 names={'(a) Initial lane-change formation','(b) Sparse formation obtained by backward translations'};text(ax,.5,-.30,names{panel},'Units','normalized','HorizontalAlignment','center','FontSize',21);ylabel(ax,'{\it y} (m)');
end
annotation(f,'textbox',[.07,.01,.9,.06],'String',['Priority order: ',strjoin(arrayfun(@num2str,priority','UniformOutput',false),' > ')],'EdgeColor','none','FontName','Times New Roman','FontSize',16,'HorizontalAlignment','center');savefigs(f,out,'fig04-04');
% 4.5: stages and lane-change intermediate motion, in one moving frame.
rel=squeeze(book.state(1,:,:))-p.v0*book.time';xl=[5*floor(min(rel(:))/5)-5,5*ceil(max(rel(:))/5)+5];T1=book.stage1_duration;T2=book.time(end)-T1;
targets=[0,.5*T1,T1,T1+T2/3,T1+2*T2/3,book.time(end)];
f=figure('Color','w','Position',[50,50,1050,1250],'Visible','off');
for panel=1:6
 ax=axes(f,'Position',[.07,.86-(panel-1)*.158,.90,.093]);hold(ax,'on');road(ax,xl,p);[~,k]=min(abs(book.time-targets(panel)));
 if panel<=3,st='Stage 1';else,st='Stage 2';end
 for i=1:p.N
  if book.time(k)>=T1-1e-8
   first=find(book.time>=T1,1);last=numel(book.time);shift=p.v0*book.time(k);
   plot(ax,squeeze(book.state(1,first:k,i))-shift,squeeze(book.state(2,first:k,i)),'-','Color',colors(i,:),'LineWidth',1.1);
   plot(ax,squeeze(book.state(1,k:last,i))-shift,squeeze(book.state(2,k:last,i)),'--','Color',colors(i,:),'LineWidth',.9);
  end
  q=book.state(:,k,i);q(1)=q(1)-p.v0*book.time(k);BookDrawVehicle(ax,q,g,colors(i,:),i,11);
 end
 text(ax,.5,-.40,sprintf('(%c) %s, t = %.2f s',96+panel,st,book.time(k)),'Units','normalized','HorizontalAlignment','center','FontSize',14.5);ylabel(ax,'{\it y} (m)');yticks(ax,[3.75,7.5,11.25]);xticks(ax,ceil(xl(1)/20)*20:20:floor(xl(2)/20)*20);set(ax,'FontSize',12);
end
savefigs(f,out,'fig04-05');
% 4.6: state/control histories including the acceleration derivative.
f=figure('Color','w','Position',[50,50,1100,1000],'Visible','off');
Y={squeeze(book.state(2,:,:)),squeeze(book.state(4,:,:)),squeeze(book.control(1,:,:)), ...
 squeeze(book.state(3,:,:)),squeeze(book.state(5,:,:)),squeeze(book.control(2,:,:)), ...
 squeeze(diff(book.control(1,:,:),1,2))./diff(book.time)',book.interpolation_validation.interval_bound(:)};
labels={'{\it y} (m)','{\it v} (m/s)','{\it a} (m/s^2)','\theta (rad)','\phi (rad)','\omega (rad/s)','\it jerk \rm(m/s^3)','Min. clearance bound (m)'};
fine=BookRefinePose(book,20);
for panel=1:8
 ax=subplot(4,2,panel);hold(ax,'on');
 if panel==7,tt=(book.time(1:end-1)+book.time(2:end))/2;elseif panel==8,tt=(fine.time(1:end-1)+fine.time(2:end))/2;else,tt=book.time;end
 if panel==8,plot(ax,tt,Y{panel},'k-','LineWidth',1.1);else
  for i=1:p.N
   if panel==7,stairs(ax,book.time,[Y{panel}(:,i);Y{panel}(end,i)],'Color',colors(i,:),'LineWidth',1);
   else,plot(ax,tt,Y{panel}(:,i),'Color',colors(i,:),'LineWidth',1);end
  end
 end
 ranges={[1.8,13.1],[8.5,11.6],[-.55,.55],[-.3,.55],[-.22,.22],[-.28,.28],[-.23,.23],[0,.45]};ylim(ax,ranges{panel});
 if panel==8,yline(ax,p.clearance,'k:');end
 xline(ax,T1,'k--','LineWidth',.7);xlim(ax,[0,book.time(end)]);xlabel(ax,'{\it t} (s)');ylabel(ax,labels{panel});grid(ax,'on');set(ax,'FontName','Times New Roman','FontSize',13);box(ax,'off');
end
for i=1:p.N
 x=.10+(i-1)*.079;annotation(f,'line',[x,x+.025],[.983,.983],'Color',colors(i,:),'LineWidth',1.6);annotation(f,'textbox',[x+.028,.971,.04,.03],'String',num2str(i),'LineStyle','none','FontName','Times New Roman','FontSize',13);
end
savefigs(f,out,'fig04-06');
% 4.7: same task, smaller jerk limit; no other-method comparison.
lo=load(fullfile(root,'output/lower_jerk/book_result.mat'));[~,id]=max(abs(book.translations));
f=figure('Color','w','Position',[50,50,1050,410],'Visible','off');
for panel=1:2
 ax=axes(f,'Position',[.08+(panel-1)*.48,.23,.38,.72]);hold(ax,'on');
 for run=1:2
  if run==1,b=book;col=colors(1,:);style='-';else,b=lo.book;col=colors(6,:);style='--';end
  k=find(b.time<=b.stage1_duration+1e-8);
  if panel==1,y=squeeze(b.control(1,k,id));lab='{\it a} (m/s^2)';else,y=squeeze(b.state(4,k,id));lab='{\it v} (m/s)';end
  plot(ax,b.time(k),y,style,'Color',col,'LineWidth',1.6);plot(ax,b.time(k(end)),y(end),'o','Color',col,'HandleVisibility','off');
 end
 xlabel(ax,'{\it t} (s)');ylabel(ax,lab);grid(ax,'on');box(ax,'off');set(ax,'FontName','Times New Roman','FontSize',15);text(ax,.5,-.24,sprintf('(%c) %s',96+panel,choosecaption(panel)),'Units','normalized','HorizontalAlignment','center','FontSize',20.5);
 legend(ax,{'jerk limit: 0.2 m/s^3','jerk limit: 0.1 m/s^3'},'Location','best','FontSize',13,'Box','off');
end
savefigs(f,out,'fig04-07');
report=struct('conflict_pair',pair,'conflict_time',blind.time(kstar),'pretranslation_minimum_sat_gap',worst,'priority',priority,'snapshots',targets,'jerk_comparison_vehicle',id);
fid=fopen(fullfile(out,'figure_provenance.json'),'w');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
function road(ax,xl,p)
patch(ax,xl([1,2,2,1]),[p.road_lower,p.road_lower,p.road_upper,p.road_upper],[.96,.96,.96],'EdgeColor','none');
for y=[p.road_lower,p.road_upper],plot(ax,xl,[y,y],'k-','LineWidth',.7);end
for y=[5.625,9.375],plot(ax,xl,[y,y],'--','Color',[.65,.65,.65],'LineWidth',.7);end
axis(ax,'equal');axis(ax,[xl,p.road_lower,p.road_upper]);set(ax,'FontName','Times New Roman','FontSize',14);box(ax,'off');
end
function circle(ax,c,r,col,w,st)
a=linspace(0,2*pi,241);plot(ax,c(1)+r*cos(a),c(2)+r*sin(a),st,'Color',col,'LineWidth',w);
end
function savefigs(f,out,n)
set(findall(f,'Type','text'),'FontName','Times New Roman','Color','k','BackgroundColor','none');
exportgraphics(f,fullfile(out,[n,'.pdf']),'ContentType','vector');exportgraphics(f,fullfile(out,[n,'.png']),'Resolution',220);close(f);
end

function s=choosecaption(panel)
names={'Acceleration','Velocity'};s=names{panel};
end
