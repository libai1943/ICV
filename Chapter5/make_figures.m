function make_figures()
root=fileparts(mfilename('fullpath'));addpath(fullfile(root,'common'));BookPlotStyle();out=fullfile(root,'output/figures');if ~isfolder(out),mkdir(out);end
chosen=15;
data=load(fullfile(root,sprintf('output/case_%02d.mat',chosen)));result=data.result;assert(result.success,'Only validated trajectories may be plotted as solutions.');scene=result.scene;states=result.trajectory;colors=repmat(BookPalette(),2,1);radius=hypot(scene.geometry.LW+scene.geometry.LF+scene.geometry.LR,scene.geometry.LB)/2;
% 5.1 is a credited real photograph in the manuscript, not a simulation.
% 5.2: exact Euclidean offset, with round vertex arcs and no artificial seams.
f=figure('Color','w','Position',[50,50,1100,550],'Visible','off');
P=[-7,3;-5,6;-1,5.3;1.3,3.5;-.5,1.5;-3,2.2];
for side=1:2
 ax=axes(f,'Position',[.035+(side-1)*.5,.17,.43,.76]);hold(ax,'on');axis(ax,'equal');axis(ax,[-10,8,-7,9]);axis(ax,'off');
 if side==2
  grown=polybuffer(polyshape(P),radius+scene.margin,'JointType','round');
  plot(ax,grown,'FaceColor',[.89,.88,.85],'FaceAlpha',1,'EdgeColor',[.58,.55,.48],'LineWidth',1.1);
 end
 patch(ax,P(:,1),P(:,2),[.65,.66,.67],'EdgeColor',[.35,.36,.37],'LineWidth',1);
 pose=[-.5;-3;.45;0;0];c=pose(1:2)+(scene.geometry.LW+scene.geometry.LF-scene.geometry.LR)/2*[cos(pose(3));sin(pose(3))];
 if side==1,BookDrawVehicle(ax,pose,scene.geometry,colors(1,:),[],16);circle(ax,c,radius,colors(1,:),1.25,'-');end
 plot(ax,c(1),c(2),'ko','MarkerFaceColor','k','MarkerSize',4);
 text(ax,c(1)-.9,c(2)+.6,'{\it c_i}','FontSize',19);text(ax,-3,4,'Obstacle','FontSize',17,'HorizontalAlignment','center');
 if side==1
  plot(ax,c(1)+[0,radius*cos(-.8)],c(2)+[0,radius*sin(-.8)],'k-','LineWidth',1.1);text(ax,c(1)+2,c(2)-2,'r_i','FontSize',19);
 else
  quiver(ax,-5.3,5.4,-1.5,1.8,0,'Color',[.35,.35,.35],'LineWidth',1,'MaxHeadSize',.22);text(ax,-3,8,'r_i + d_{safe}','FontSize',18,'HorizontalAlignment','center');
 end
 names={'(a) Enclosing disc','(b) Inflated obstacle'};text(ax,-1,-8.1,names{side},'FontSize',18,'HorizontalAlignment','center');
end
save_plot(f,out,'fig05-02');
% 5.3: one coordinate transformation, shown with matching scales and symbols.
f=figure('Color','w','Position',[50,50,1100,565],'Visible','off');
ax=axes(f,'Position',[.035,.17,.43,.76]);hold(ax,'on');axis(ax,'equal');axis(ax,[-10,11,-8,11]);axis(ax,'off');
cent=[3,-3;2,-2];angles=[.55,2.65];offset=(scene.geometry.LW+scene.geometry.LF-scene.geometry.LR)/2;
for j=1:2
 pose=[cent(:,j)-offset*[cos(angles(j));sin(angles(j))];angles(j);0;0];BookDrawVehicle(ax,pose,scene.geometry,colors(j*3,:),[],16);
 circle(ax,cent(:,j),radius,colors(j*3,:),1.2,'-');plot(ax,cent(1,j),cent(2,j),'k.','MarkerSize',12);
end
quiver(ax,-3,-2,6,4,0,'Color',[.18,.18,.18],'LineWidth',1.3,'MaxHeadSize',.2);
text(ax,-3,-5.5,'c_j','FontSize',19,'HorizontalAlignment','center');text(ax,3,5.3,'c_i','FontSize',19,'HorizontalAlignment','center');text(ax,-.8,1.7,'d_{ij}','FontSize',19);
text(ax,.5,-9.2,'(a) Absolute coordinates','FontSize',18,'HorizontalAlignment','center');
ax=axes(f,'Position',[.535,.17,.43,.76]);hold(ax,'on');axis(ax,'equal');axis(ax,[-8,13,-8,11]);axis(ax,'off');limit=2*radius+scene.margin;
angle=linspace(0,2*pi,241);patch(ax,limit*cos(angle),limit*sin(angle),[.86,.85,.83],'EdgeColor',[.45,.44,.42],'LineWidth',1);
patch(ax,[5.5,10,10,5.5],[1,1,7,7],[.88,.93,.95],'EdgeColor',colors(3,:),'LineWidth',1.4);
plot(ax,[0,5.5],[0,1],'k:','LineWidth',.9);plot(ax,5.5,1,'ko','MarkerFaceColor','w','MarkerSize',5);
quiver(ax,0,0,6,4,0,'Color',[.18,.18,.18],'LineWidth',1.3,'MaxHeadSize',.2);plot(ax,6,4,'ko','MarkerFaceColor','w','MarkerSize',6);
text(ax,.1,-.9,'0','FontSize',18);text(ax,6.6,3.5,'Seed','FontSize',17);text(ax,7.1,8.2,'Relative corridor','FontSize',15,'HorizontalAlignment','center');
plot(ax,[-limit,0],[0,0],'k-','LineWidth',.7);text(ax,-2.8,-1.8,'r_i + r_j + d_{safe}','FontSize',15,'HorizontalAlignment','center');
text(ax,2.5,-9.2,'(b) Relative coordinates','FontSize',18,'HorizontalAlignment','center');save_plot(f,out,'fig05-03');
% 5.4: directions follow explicitly drawn center/nearest-boundary geometry.
[f,ax]=panel(1100,680);axis(ax,[-9,12,-6,9]);axis(ax,'off');
P=[-8,3;-6,6;-3,6.7;-1,4.4;-2.8,2.8;-5.5,2.3];patch(ax,P(:,1),P(:,2),[.74,.75,.76],'EdgeColor',[.4,.4,.4],'LineWidth',1);
ci=[0;0];cj=[6;3.5];theta=.25;otherTheta=-2.5;
BookDrawVehicle(ax,[ci-offset*[cos(theta);sin(theta)];theta;0;0],scene.geometry,colors(2,:),[],16);
BookDrawVehicle(ax,[cj-offset*[cos(otherTheta);sin(otherTheta)];otherTheta;0;0],scene.geometry,colors(7,:),[],16);
ends=P([2:end,1],:);edges=ends-P;fraction=max(0,min(1,sum((ci'-P).*edges,2)./sum(edges.^2,2)));points=P+fraction.*edges;[~,nearest]=min(vecnorm(points-ci',2,2));foot=points(nearest,:)';
plot(ax,[ci(1),cj(1)],[ci(2),cj(2)],'k:','LineWidth',.9);plot(ax,[ci(1),foot(1)],[ci(2),foot(2)],'k:','LineWidth',.9);plot(ax,foot(1),foot(2),'ks','MarkerSize',5,'MarkerFaceColor','w');
guide=[4.1;2.2];repel=3*(ci-cj)/norm(ci-cj);wall=3*(ci-foot)/norm(ci-foot);vectors=[guide,repel,wall,guide+repel+wall];cols=[colors(4,:);colors(8,:);colors(1,:);.12,.12,.12];
for j=1:4,v=vectors(:,j);quiver(ax,0,0,v(1),v(2),0,'Color',cols(j,:),'LineWidth',1.7,'MaxHeadSize',.22);end
text(ax,4.1,1.5,'f_{guide}','FontSize',19,'Color',cols(1,:));text(ax,-5,-1.7,'f_{vehicle}','FontSize',19,'Color',cols(2,:));text(ax,2.1,-3.7,'f_{boundary}','FontSize',19,'Color',cols(3,:));text(ax,5.0,-1.6,'f_{total}','FontSize',19);
text(ax,-5.3,4.5,'Boundary','FontSize',17,'HorizontalAlignment','center');text(ax,5.8,6.3,'Neighbor j','FontSize',17,'HorizontalAlignment','center');text(ax,-1.2,-.7,'c_i','FontSize',17);text(ax,foot(1)-.3,foot(2)+.6,'Nearest point','FontSize',16,'HorizontalAlignment','right');
save_plot(f,out,'fig05-04');
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
% 5.6: six snapshots from the computed solution.
f=figure('Color','w','Position',[50,50,900,1230],'Visible','off');fractions=[0,.18,.32,.46,.65,1];
for panelIndex=1:6
 ax=axes(f,'Position',[.025+mod(panelIndex-1,2)*.50,.72-floor((panelIndex-1)/2)*.33,.45,.26]);hold(ax,'on');draw_map(ax,scene,[-73,73,-73,73]);k=1+round(fractions(panelIndex)*(scene.nodes-1));
 for car=1:chosen
  plot(ax,states(1,1:k,car),states(2,1:k,car),'-','Color',colors(car,:),'LineWidth',1.1);
  BookDrawVehicle(ax,states(:,k,car),scene.geometry,colors(car,:),car,5.5);
 end
 axis(ax,'off');text(ax,0,-82,sprintf('(%c) t = %.1f s',96+panelIndex,(k-1)*scene.horizon/(scene.nodes-1)),'FontSize',18,'HorizontalAlignment','center');
end
save_plot(f,out,'fig05-06');
% 5.7: actual state/control profiles and independent clearance.
f=figure('Color','w','Position',[50,50,1080,750],'Visible','off');time=linspace(0,scene.horizon,scene.nodes);captions={'(a) Velocity','(b) Steering angle','(c) Jerk','(d) Full-geometry clearance'};units={'v (m/s)','\phi (rad)','jerk (m/s^3)','Clearance (m)'};rows=[4,5,7];
for index=1:4
 ax=axes(f,'Position',[.085+mod(index-1,2)*.49,.62-floor((index-1)/2)*.49,.38,.29]);hold(ax,'on');
 if index<4,for car=1:chosen,plot(ax,time,states(rows(index),:,car),'Color',colors(car,:),'LineWidth',1);end
 else,plot(ax,linspace(0,scene.horizon,numel(result.validation.clearance)),result.validation.clearance,'Color',[.2,.2,.2],'LineWidth',1);yline(ax,0,'k:');end
 xlabel(ax,'t (s)');ylabel(ax,units{index});grid(ax,'on');box(ax,'off');text(ax,.5,-.34,captions{index},'Units','normalized','HorizontalAlignment','center','FontSize',18);
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
