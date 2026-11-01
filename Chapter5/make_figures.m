function make_figures()
root=fileparts(mfilename('fullpath'));addpath(fullfile(root,'common'));BookPlotStyle();out=fullfile(root,'output/figures');if ~isfolder(out),mkdir(out);end
chosen=15;
data=load(fullfile(root,sprintf('output/case_%02d.mat',chosen)));result=data.result;assert(result.success,'Only validated trajectories may be plotted as solutions.');scene=result.scene;states=result.trajectory;colors=[BookPalette();.30,.57,.73;.45,.66,.48;.72,.54,.25;.65,.38,.41;.51,.45,.65];
% 5.1 is a credited real photograph in the manuscript, not a simulation.
% Concept diagrams are editable vector artwork supplied with the project.
for number=2:4,copyfile(fullfile(root,'assets',sprintf('fig05-%02d.pdf',number)),out);end
% 5.5: illustrative timing, explicitly not a measured throughput experiment.
[f,ax]=panel(1050,460);axis(ax,'normal');axis(ax,[0,32,.3,3.7]);set(ax,'YTick',1:3,'YTickLabel',{'Batch 3','Batch 2','Batch 1'},'XTick',0:5:30);xlabel(ax,'Illustrative time');hold(ax,'on');
stages={'Preparation','Reservation','Traversal','Cruising'};c=[.87,.88,.89;.80,.87,.93;.54,.71,.82;.85,.80,.66];
for row=1:3
 shift=(3-row)*5;edges=[0,5,10,16,21]+shift;
 for k=1:4
  patch(ax,edges([k,k+1,k+1,k]),row+[-.25,-.25,.25,.25],c(k,:),'EdgeColor','w');
  text(ax,mean(edges(k:k+1)),row,stages{k},'FontSize',14,'HorizontalAlignment','center');
 end
end
box(ax,'off');save_plot(f,out,'fig05-05');
% 5.6: overview endpoints and four central snapshots from the actual solution.
f=figure('Color','w','Position',[50,50,900,1120],'Visible','off');
centers=disc_centers(states,scene);body=squeeze(mean(centers,3));occupancy=sum(squeeze((body(1,:,:)/12).^2+(body(2,:,:)/9).^2)<=1,2);[~,peak]=max(occupancy);
dt=scene.horizon/(scene.nodes-1);[~,bendIndex]=max(max(abs(states(5,:,:)),[],3));
middle=unique([max(2,peak-round(5/dt)),peak,min(scene.nodes-1,peak+round(6/dt)),bendIndex]);
for extra=peak+round([3,-3,9,-9]/dt),if numel(middle)<4&&extra>1&&extra<scene.nodes,middle=unique([middle,extra]);end;end
middle=sort(middle);assert(numel(middle)==4);indices=[1,middle(:)',scene.nodes];
for panelIndex=1:6
 ax=axes(f,'Position',[.025+mod(panelIndex-1,2)*.50,.72-floor((panelIndex-1)/2)*.33,.45,.265]);hold(ax,'on');
 if ismember(panelIndex,[1,6]),limits=[-85,85,-85,85];bar=20;else,limits=[-33,33,-33,33];bar=5;end
 draw_map(ax,scene,limits);k=indices(panelIndex);
 for car=1:chosen
  plot(ax,states(1,1:k,car),states(2,1:k,car),'-','Color',colors(car,:),'LineWidth',.9);
  BookDrawVehicle(ax,states(:,k,car),scene.geometry,colors(car,:),[],6);
 end
 axis(ax,'off');span=limits(2)-limits(1);x=limits(1)+.06*span;y=limits(3)+.07*span;plot(ax,x+[0,bar],[y,y],'k-','LineWidth',1.1);text(ax,x+bar/2,y+.04*span,sprintf('%d m',bar),'FontSize',12,'HorizontalAlignment','center');
 text(ax,.5,-.035,sprintf('(%c) t = %.1f s',96+panelIndex,(k-1)*dt),'Units','normalized','FontSize',18,'HorizontalAlignment','center');
end
save_plot(f,out,'fig05-06');
% 5.9: enlarged central interaction, with true steering and complete trajectories.
[f,ax]=panel(1050,730);draw_map(ax,scene,[-24,24,-18,18]);k=peak;
for car=1:chosen
 plot(ax,states(1,:,car),states(2,:,car),':','Color',.5+.5*colors(car,:),'LineWidth',.7);
 from=max(1,k-round(8/dt));plot(ax,states(1,from:k,car),states(2,from:k,car),'-','Color',colors(car,:),'LineWidth',1.5);
 BookDrawVehicle(ax,states(:,k,car),scene.geometry,colors(car,:),[],8);
end
axis(ax,'off');plot(ax,[-21,-16],[-15,-15],'k-','LineWidth',1.2);text(ax,-18.5,-13.7,'5 m','HorizontalAlignment','center','FontSize',17);
text(ax,-21,16,sprintf('t = %.1f s; %d ICVs in the central region',(k-1)*dt,max(occupancy)),'FontSize',17,'HorizontalAlignment','left');
save_plot(f,out,'fig05-09');
% 5.7: actual state/control profiles and independent clearance.
f=figure('Color','w','Position',[50,50,1080,750],'Visible','off');time=linspace(0,scene.horizon,scene.nodes);captions={'(a) Velocity','(b) Steering angle','(c) Jerk','(d) Full-geometry clearance'};units={'{\it v} (m/s)','\phi (rad)','{\it jerk} (m/s^3)','Clearance (m)'};rows=[4,5,7];
for index=1:4
 ax=axes(f,'Position',[.085+mod(index-1,2)*.49,.62-floor((index-1)/2)*.49,.38,.29]);hold(ax,'on');
 if index<4,for car=1:chosen,plot(ax,time,states(rows(index),:,car),'Color',colors(car,:),'LineWidth',1);end
 else,plot(ax,linspace(0,scene.horizon,numel(result.validation.clearance)),result.validation.clearance,'Color',[.2,.2,.2],'LineWidth',1);yline(ax,0,'k:');end
 xlabel(ax,'{\it t} (s)');ylabel(ax,units{index});grid(ax,'on');box(ax,'off');text(ax,.5,-.34,captions{index},'Units','normalized','HorizontalAlignment','center','FontSize',18);
end
save_plot(f,out,'fig05-07');
% 5.8: actual maximum motion defects during each tested run.
[f,ax]=panel(900,470);axis(ax,'normal');hold(ax,'on');for count=[9,12,15]
 file=fullfile(root,sprintf('output/case_%02d.mat',count));if ~isfile(file),continue;end
 r=load(file);h=r.result.history;semilogy(ax,[h.iteration],[h.maximum_defect],'-o','LineWidth',1.4,'DisplayName',sprintf('%d ICVs',count));
end
set(ax,'YScale','log');xlabel(ax,'Outer iteration');ylabel(ax,'Maximum transcription defect');grid(ax,'on');legend(ax,'Location','best','Box','off');save_plot(f,out,'fig05-08');
end
function [f,ax]=panel(w,h)
f=figure('Color','w','Position',[50,50,w,h],'Visible','off');ax=axes(f,'Position',[.075,.12,.88,.82]);hold(ax,'on');axis(ax,'equal');
end
function draw_map(ax,scene,limits)
for k=1:numel(scene.obstacles),P=scene.obstacles{k};patch(ax,P(:,1),P(:,2),[.84,.85,.86],'EdgeColor',[.65,.66,.67],'LineWidth',.7);end
axis(ax,'equal');axis(ax,limits);
end
function circle(ax,c,r,color,width,style)
angle=linspace(0,2*pi,161);plot(ax,c(1)+r*cos(angle),c(2)+r*sin(angle),style,'Color',color,'LineWidth',width);
end
function save_plot(f,out,name)
set(findall(f,'Type','axes'),'FontName','Times New Roman','FontSize',14);set(findall(f,'Type','text'),'FontName','Times New Roman','BackgroundColor','none');
exportgraphics(f,fullfile(out,[name,'.pdf']),'ContentType','vector');exportgraphics(f,fullfile(out,[name,'.png']),'Resolution',180);close(f);
end
