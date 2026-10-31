function make_figures()
root=fileparts(mfilename('fullpath'));addpath(fullfile(root,'common'));BookPlotStyle();out=fullfile(root,'output/figures');if ~isfolder(out),mkdir(out);end
chosen=12;
data=load(fullfile(root,sprintf('output/case_%02d.mat',chosen)));result=data.result;assert(result.success,'Only validated trajectories may be plotted as solutions.');scene=result.scene;states=result.trajectory;colors=[BookPalette();.45,.46,.2;.45,.25,.23];radius=hypot(scene.geometry.LW+scene.geometry.LF+scene.geometry.LR,scene.geometry.LB)/2;
% 5.1: a computed close view of the active intersection.
[f,ax]=panel(880,780);draw_map(ax,scene,[-32,32,-32,32]);k=1+round(.32*(scene.nodes-1));
for car=1:chosen
 plot(ax,states(1,1:k,car),states(2,1:k,car),'-','Color',colors(car,:),'LineWidth',1.2);
 if all(abs(states(1:2,k,car))<30),BookDrawVehicle(ax,states(:,k,car),scene.geometry,colors(car,:),car,14);end
end
axis(ax,'off');save_plot(f,out,'fig05-01');
% 5.2: radius inflation; circles create rounded offset boundaries.
f=figure('Color','w','Position',[50,50,1050,530],'Visible','off');
for side=1:2
 ax=axes(f,'Position',[.03+(side-1)*.5,.12,.45,.82]);hold(ax,'on');axis(ax,'equal');axis(ax,[-15,15,-15,15]);axis(ax,'off');
 for k=1:4
  P=scene.obstacles{k};
  if side==2
   angles=linspace(0,2*pi,81);cloud=[];for vertex=1:size(P,1),cloud=[cloud;P(vertex,:)+(radius+scene.margin)*[cos(angles)',sin(angles)']];end
   hull=convhull(cloud(:,1),cloud(:,2));patch(ax,cloud(hull,1),cloud(hull,2),[.87,.88,.89],'EdgeColor',[.7,.7,.7],'LineWidth',.6);
  end
  patch(ax,P(:,1),P(:,2),[.58,.60,.61],'EdgeColor',[.4,.4,.4]);
 end
 pose=[-2.5;-2.9;0;0;0];center=pose(1:2)+(scene.geometry.LW+scene.geometry.LF-scene.geometry.LR)/2*[1;0];
 if side==1,BookDrawVehicle(ax,pose,scene.geometry,colors(1,:),[],16);circle(ax,center,radius,colors(1,:),1.2,'-');else,plot(ax,center(1),center(2),'ko','MarkerFaceColor','k');end
 names={'(a) Vehicle and physical boundary','(b) Center and inflated boundary'};text(ax,0,-17,names{side},'FontSize',18,'HorizontalAlignment','center');
end
save_plot(f,out,'fig05-02');
% 5.3: an actual relative corridor, using the same verified vehicle pair.
centers=states(1:2,:,:)+(scene.geometry.LW+scene.geometry.LF-scene.geometry.LR)/2*cat(1,cos(states(3,:,:)),sin(states(3,:,:)));
best=inf;pair=[1,2];instant=1;
for i=1:chosen-1,for j=i+1:chosen,[distance,k]=min(vecnorm(centers(:,:,i)-centers(:,:,j)));if distance<best,best=distance;pair=[i,j];instant=k;end,end,end
boxes=construct_corridors(scene,states);index=find(all(boxes.pairs==pair,2));delta=centers(:,:,pair(1))-centers(:,:,pair(2));seed=delta(:,instant);
if isempty(index),relativeBox=[seed(1)-.1;seed(1)+.1;seed(2)-.1;seed(2)+.1];else,relativeBox=boxes.relative(:,instant,index);end
f=figure('Color','w','Position',[50,50,1100,580],'Visible','off');
ax=axes(f,'Position',[.03,.14,.44,.82]);hold(ax,'on');axis(ax,'equal');mid=mean(centers(:,instant,pair),3);
for id=pair
 BookDrawVehicle(ax,states(:,instant,id),scene.geometry,colors(id,:),id,16);circle(ax,centers(:,instant,id),radius,colors(id,:),1.1,'-');plot(ax,centers(1,instant,id),centers(2,instant,id),'k.');
end
a=centers(:,instant,pair(2));b=centers(:,instant,pair(1));quiver(ax,a(1),a(2),b(1)-a(1),b(2)-a(2),0,'Color',[.2,.2,.2],'LineWidth',1);
text(ax,mid(1)+.3,mid(2)+.5,'d_{ij}','FontSize',19);axis(ax,[mid(1)-12,mid(1)+12,mid(2)-12,mid(2)+12]);axis(ax,'off');text(ax,mid(1),mid(2)-13.8,'(a) Absolute vehicle geometry','FontSize',19,'HorizontalAlignment','center');
ax=axes(f,'Position',[.53,.14,.44,.82]);hold(ax,'on');axis(ax,'equal');angles=linspace(0,2*pi,181);limit=2*radius+scene.margin;
patch(ax,limit*cos(angles),limit*sin(angles),[.83,.84,.85],'EdgeColor',[.45,.45,.45]);
patch(ax,relativeBox([1,2,2,1]),relativeBox([3,3,4,4]),[.88,.93,.98],'FaceAlpha',.7,'EdgeColor',colors(1,:),'LineWidth',1.3);
window=max(1,instant-12):min(scene.nodes,instant+12);plot(ax,delta(1,window),delta(2,window),'--','Color',colors(1,:),'LineWidth',1.1);plot(ax,seed(1),seed(2),'ko','MarkerFaceColor','w','MarkerSize',6);
quiver(ax,0,0,seed(1),seed(2),0,'Color',[.3,.3,.3],'LineWidth',.9);text(ax,.2,.5,'0','FontSize',18);text(ax,seed(1)+.6,seed(2)+.4,'Seed','FontSize',18);
plot(ax,[0,limit],[0,0],'k-');text(ax,limit/2,-.8,'r_i + r_j + d_{safe}','FontSize',16,'HorizontalAlignment','center');
axis(ax,[-10,14,-7,17]);axis(ax,'off');text(ax,2,-8.8,'(b) Relative forbidden disc and box','FontSize',19,'HorizontalAlignment','center');
save_plot(f,out,'fig05-03');
% 5.4: SFM vector diagram with the established rectangular vehicle glyph.
[f,ax]=panel(900,610);axis(ax,[-7,9,-5,6]);axis(ax,'off');pose=[0;0;.35;0;0];BookDrawVehicle(ax,pose,scene.geometry,colors(1,:),[],16);BookDrawVehicle(ax,[5;3;-.5;0;0],scene.geometry,colors(6,:),[],16);
P=[-5,2;-4.5,3.8;-2.8,4.8;-1.7,4.2;-1.9,2.8;-3.3,2.1];patch(ax,P(:,1),P(:,2),[.75,.76,.77],'EdgeColor',[.4,.4,.4]);
center=(scene.geometry.LW+scene.geometry.LF-scene.geometry.LR)/2*[cos(.35);sin(.35)];
other=[5;3]+(scene.geometry.LW+scene.geometry.LF-scene.geometry.LR)/2*[cos(-.5);sin(-.5)];
ends=P([2:end,1],:);edges=ends-P;fraction=max(0,min(1,sum((center'-P).*edges,2)./sum(edges.^2,2)));points=P+fraction.*edges;[~,nearest]=min(vecnorm(points-center',2,2));away=center-points(nearest,:)';
repulsion=2.2*(center-other)/norm(center-other);boundary=2*away/norm(away);guidance=[3.2;2.1];vectors=[guidance,repulsion,boundary,guidance+repulsion+boundary];
labels={'Guidance','Vehicle repulsion','Boundary repulsion','Resultant'};cols=[colors(3,:);colors(7,:);colors(2,:);.15,.15,.15];positions=[5.3,4.9;-2.2,-1.7;4.3,-1.7;6.4,.2];
for k=1:4,v=vectors(:,k);quiver(ax,center(1),center(2),v(1),v(2),0,'Color',cols(k,:),'LineWidth',1.5,'MaxHeadSize',.25);text(ax,positions(k,1),positions(k,2),labels{k},'FontSize',18,'HorizontalAlignment','center','Color',cols(k,:));end
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
 ax=axes(f,'Position',[.025+mod(panelIndex-1,2)*.50,.72-floor((panelIndex-1)/2)*.33,.45,.26]);hold(ax,'on');draw_map(ax,scene,[-47,47,-47,47]);k=1+round(fractions(panelIndex)*(scene.nodes-1));
 for car=1:chosen
  plot(ax,states(1,1:k,car),states(2,1:k,car),'-','Color',colors(car,:),'LineWidth',.8);
  BookDrawVehicle(ax,states(:,k,car),scene.geometry,colors(car,:),car,9);
 end
 axis(ax,'off');text(ax,0,-54,sprintf('(%c) t = %.1f s',96+panelIndex,(k-1)*scene.horizon/(scene.nodes-1)),'FontSize',18,'HorizontalAlignment','center');
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
[f,ax]=panel(900,470);axis(ax,'normal');hold(ax,'on');for count=[4,8,12]
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
