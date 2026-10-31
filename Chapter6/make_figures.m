function make_figures()
root=fileparts(mfilename('fullpath'));addpath(fullfile(root,'common'));BookPlotStyle();out=fullfile(root,'output/figures');if ~isfolder(out),mkdir(out);end
data=load(fullfile(root,'output/case_01_asco.mat'));r=data.result;assert(r.success,'A validated demonstration is required.');task=r.task;colors=BookPalette();count=size(r.state,3);
f=figure('Color','w','Position',[50,50,1000,540],'Visible','off');
for panel=1:2
 ax=axes(f,'Position',[.025+(panel-1)*.5,.13,.45,.83]);hold(ax,'on');map(ax,task);
 for car=1:count
  if panel==1,pose=task.start(:,car);else,pose=task.goal(:,car);end
  BookDrawVehicle(ax,pose,task.geometry,colors(car,:),car,12);
 end
 axis(ax,'off');names={'(a) Initial configuration','(b) Terminal configuration'};text(ax,0,-26,names{panel},'FontSize',18,'HorizontalAlignment','center');
end
saveplot(f,out,'fig06-04');
% Signed-gap illustration in disc coordinates; not an experimental snapshot.
f=figure('Color','w','Position',[50,50,1050,400],'Visible','off');gaps=[-1.35,.2,3.2];captions={'(a) Inner exclusion','(b) Selected interaction','(c) Distant interaction'};
for panel=1:3
 ax=axes(f,'Position',[.015+(panel-1)*.33,.23,.31,.69]);hold(ax,'on');axis(ax,'equal');axis(ax,[-1.5,7,-2,2]);axis(ax,'off');d=2+gaps(panel);
 circle(ax,[0;0],1,[.6,.63,.66]);circle(ax,[d;0],1,colors(panel*2,:));plot(ax,[0,d],[0,0],'k:');plot(ax,[0,d],[0,0],'k.');
 text(ax,2.75,-1.65,sprintf('s = %.2f m',gaps(panel)),'FontSize',18,'HorizontalAlignment','center');text(ax,2.75,-2.6,captions{panel},'FontSize',18,'HorizontalAlignment','center');
end
annotation(f,'textbox',[.05,.01,.9,.12],'String','Illustrative thresholds: S_{LB} = -0.5 m, S_{UB} = 2 m; disc radii = 1 m','EdgeColor','none','HorizontalAlignment','center','FontName','Times New Roman','FontSize',16);
saveplot(f,out,'fig06-03');
f=figure('Color','w','Position',[50,50,900,1230],'Visible','off');fractions=[0,.18,.36,.55,.75,1];
for panel=1:6
 ax=axes(f,'Position',[.025+mod(panel-1,2)*.50,.72-floor((panel-1)/2)*.33,.45,.26]);hold(ax,'on');map(ax,task);k=1+round(fractions(panel)*(size(r.state,2)-1));
 for car=1:count
  plot(ax,r.state(1,1:k,car),r.state(2,1:k,car),'Color',colors(car,:),'LineWidth',1.3);BookDrawVehicle(ax,r.state(:,k,car),task.geometry,colors(car,:),car,10);
 end
 axis(ax,'off');text(ax,0,-27,sprintf('(%c) t = %.1f s',96+panel,(k-1)*r.horizon/(size(r.state,2)-1)),'FontSize',18,'HorizontalAlignment','center');
end
saveplot(f,out,'fig06-05');
% Independent dense reconstruction for the plotted clearance profile.
nodes=size(r.state,2);dt=r.horizon/(nodes-1);sub=40;dense=zeros(6,(nodes-1)*sub+1,count);
for car=1:count
 dense(:,1,car)=r.state(:,1,car);
 for k=1:nodes-1,for j=1:sub,at=(k-1)*sub+j;dense(:,at+1,car)=r.state(:,k,car)+(j/sub)*(r.state(:,k+1,car)-r.state(:,k,car));end,end
end
[squared,radius]=collision_vector(dense,task);clearance=min(sqrt(max(0,squared))-radius,[],2);time=linspace(0,r.horizon,nodes);
f=figure('Color','w','Position',[50,50,1080,750],'Visible','off');labels={'(a) Velocity','(b) Steering angle','(c) Jerk','(d) Full-geometry clearance'};units={'v (m/s)','\phi (rad)','jerk (m/s^3)','Clearance (m)'};
for panel=1:4
 ax=axes(f,'Position',[.085+mod(panel-1,2)*.49,.62-floor((panel-1)/2)*.49,.38,.29]);hold(ax,'on');
 if panel<4
  for car=1:count
   if panel==1,values=r.state(4,:,car);elseif panel==2,values=r.state(5,:,car);else,values=[r.control(1,:,car),0];end
   plot(ax,time,values,'Color',colors(car,:),'LineWidth',1);
  end
 else,plot(ax,linspace(0,r.horizon,numel(clearance)),clearance,'k-','LineWidth',1);yline(ax,0,'k:');end
 xlabel(ax,'t (s)');ylabel(ax,units{panel});grid(ax,'on');box(ax,'off');text(ax,.5,-.34,labels{panel},'Units','normalized','HorizontalAlignment','center','FontSize',18);
end
saveplot(f,out,'fig06-06');
methods={'asco','full','progressive','linearized'};names={'ASCO','Full NLP','Progressive','Linearized'};success=zeros(1,4);elapsed=nan(8,4);passed=false(8,4);
for m=1:4,for n=1:8
 file=fullfile(root,sprintf('output/case_%02d_%s.json',n,methods{m}));assert(isfile(file),'Complete all benchmark cases before drawing the comparison.');entry=jsondecode(fileread(file));elapsed(n,m)=entry.wall_seconds;passed(n,m)=entry.success;
end;success(m)=sum(passed(:,m));end
f=figure('Color','w','Position',[50,50,1120,540],'Visible','off');ax=axes(f,'Position',[.07,.23,.40,.70]);bar(ax,success,'FaceColor',[.42,.61,.72]);set(ax,'XTick',1:4,'XTickLabel',{'ASCO','Full NLP','Prog.','Lin.'});ylim(ax,[0,8.7]);ylabel(ax,'Validated cases (out of 8)');grid(ax,'on');text(ax,.5,-.23,'(a) Success count','Units','normalized','HorizontalAlignment','center','FontSize',18);
ax=axes(f,'Position',[.57,.23,.40,.70]);hold(ax,'on');
for m=1:4
 for n=1:8
  x=m+.11*(n-4.5)/4;mark='x';if passed(n,m),mark='o';end
  plot(ax,x,elapsed(n,m),mark,'Color',colors(m*2,:),'MarkerSize',7,'LineWidth',1.1);
 end
 plot(ax,m+[-.18,.18],median(elapsed(:,m))*[1,1],'k-','LineWidth',1.5);
end
set(ax,'XTick',1:4,'XTickLabel',{'ASCO','Full NLP','Prog.','Lin.'});xlim(ax,[.5,4.5]);ylabel(ax,'Elapsed wall time (s)');grid(ax,'on');text(ax,.5,-.23,'(b) Runs and median; x denotes failure','Units','normalized','HorizontalAlignment','center','FontSize',18);saveplot(f,out,'fig06-07');
f=figure('Color','w','Position',[50,50,900,470],'Visible','off');ax=axes(f,'Position',[.12,.16,.83,.79]);hold(ax,'on');
for m=[1,3]
 data=load(fullfile(root,sprintf('output/case_01_%s.mat',methods{m})));h=data.result.history;plot(ax,[h.outer_iteration],[h.loaded_collision_constraints],'-o','Color',colors(m*2,:),'DisplayName',names{m},'LineWidth',1.5);
end
xlabel(ax,'Outer iteration');ylabel(ax,'Loaded collision rows');legend(ax,'Location','best','Box','off');grid(ax,'on');saveplot(f,out,'fig06-08');
end
function map(ax,task)
for k=1:size(task.obstacles,1),circle(ax,task.obstacles(k,1:2)',task.obstacles(k,3),[.65,.67,.68]);end
axis(ax,'equal');axis(ax,[-24,24,-24,24]);
end
function circle(ax,c,r,color)
a=linspace(0,2*pi,121);patch(ax,c(1)+r*cos(a),c(2)+r*sin(a),.7+.3*color,'EdgeColor',color,'LineWidth',1);
end
function saveplot(f,out,name)
set(findall(f,'Type','axes'),'FontName','Times New Roman','FontSize',14);set(findall(f,'Type','text'),'FontName','Times New Roman','BackgroundColor','none');exportgraphics(f,fullfile(out,[name,'.pdf']),'ContentType','vector');exportgraphics(f,fullfile(out,[name,'.png']),'Resolution',180);close(f);
end
