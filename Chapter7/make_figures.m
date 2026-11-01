function make_figures()
root=fileparts(mfilename('fullpath'));addpath(fullfile(root,'common'));BookPlotStyle();out=fullfile(root,'output/figures');if ~isfolder(out),mkdir(out);end
d=load(fullfile(root,'output/formation_50.mat'));r=d.result;assert(r.success,'A complete, independently validated formation is required.');q=r.trajectory;t=r.task;nv=size(q,3);K=size(q,2);time=(0:K-1)*t.dt;colors=BookPalette();colors=colors(mod(0:nv-1,size(colors,1))+1,:);
x=q(1,:,:);y=q(2,:,:);range=[min(x,[],'all')-6,max(x,[],'all')+6,min(y,[],'all')-6,max(y,[],'all')+6];
% The overview and the introductory illustration share the same solved data.
for style=1:2
 if style==1,fractions=[0,.2,.4,.6,.8,1];h=1200;else,fractions=[0,.5,1];h=1050;end
 f=figure('Visible','off','Color','w','Position',[50,50,1100,h]);
 for panel=1:numel(fractions)
  if style==1,pos=[.03+mod(panel-1,2)*.5,.72-floor((panel-1)/2)*.33,.44,.25];else,pos=[.08,.72-(panel-1)*.33,.84,.25];end
  ax=axes(f,'Position',pos);hold(ax,'on');axis(ax,'equal');axis(ax,range);k=1+round(fractions(panel)*(K-1));
  for i=1:nv
   if style==1,first=1;else,first=max(1,k-14);end
   plot(ax,q(1,first:k,i),q(2,first:k,i),'-','Color',.22+.78*colors(i,:),'LineWidth',.65);
   if style==2,last=min(K,k+10);plot(ax,q(1,k:last,i),q(2,k:last,i),'--','Color',.22+.78*colors(i,:),'LineWidth',.55);end
  end
  for i=1:nv,BookDrawVehicle(ax,q(:,k,i),t.geometry,colors(i,:),[],7);end
  axis(ax,'off');text(ax,mean(range(1:2)),range(3)-.07*diff(range(3:4)),sprintf('(%c) t = %.1f s',96+panel,time(k)),'HorizontalAlignment','center','FontSize',18);
 end
 if style==1,name='fig07-01';else,name='fig01-03';end
 saveplot(f,out,name);
end
f=figure('Visible','off','Color','w','Position',[50,50,1080,780]);labels={'(a) Velocity','(b) Steering angle','(c) Acceleration','(d) Steering rate'};units={'{\it v} (m/s)','\phi (rad)','{\it a} (m/s^2)','\omega (rad/s)'};
for panel=1:4
 ax=axes(f,'Position',[.085+mod(panel-1,2)*.49,.62-floor((panel-1)/2)*.49,.38,.29]);hold(ax,'on');
 for i=1:nv,plot(ax,time,q(panel+3,:,i),'Color',colors(i,:),'LineWidth',.7);end
 xlabel(ax,'{\it t} (s)');ylabel(ax,units{panel});grid(ax,'on');text(ax,.5,-.34,labels{panel},'Units','normalized','HorizontalAlignment','center','FontSize',18);
end
saveplot(f,out,'fig07-02');
hist=r.history;f=figure('Visible','off','Color','w','Position',[50,50,1080,510]);
ax=axes(f,'Position',[.08,.23,.39,.70]);hold(ax,'on');semilogy(ax,1:numel(hist),[hist.maximum_goal_error],'-o','Color',colors(1,:),'MarkerSize',4);xlabel(ax,'Local solve');ylabel(ax,'Maximum terminal position error (m)');grid(ax,'on');text(ax,.5,-.25,'(a) Progress toward assigned goals','Units','normalized','HorizontalAlignment','center','FontSize',17);
ax=axes(f,'Position',[.59,.23,.37,.70]);plot(ax,1:numel(hist),[hist.selected_pairs],'-o','Color',colors(5,:),'MarkerSize',4);hold(ax,'on');yline(ax,nv*(nv-1)/2,'k--');xlabel(ax,'Local solve');ylabel(ax,'Distinct retained pairs');grid(ax,'on');text(ax,.5,-.25,'(b) Screened interaction sets','Units','normalized','HorizontalAlignment','center','FontSize',17);saveplot(f,out,'fig07-03');
end
function saveplot(f,out,name)
BookPlotStyle(f);set(findall(f,'Type','axes'),'FontSize',14);exportgraphics(f,fullfile(out,[name,'.pdf']),'ContentType','vector');exportgraphics(f,fullfile(out,[name,'.png']),'Resolution',180);close(f);
end
