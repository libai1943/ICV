function make_lane_change_figure()
root=fileparts(mfilename('fullpath'));addpath(fullfile(root,'common'));BookPlotStyle();colors=BookPalette();out=fullfile(root,'intro_results');s=load(fullfile(root,'../Chapter4/output/baseline/book_result.mat'));book=s.book;
fig=figure('Color','w','Position',[50,50,1100,1300]);
relative=squeeze(book.state(1,:,:))-10*book.time';xmin=10*floor(min(relative(:))/10)-5;xmax=10*ceil(max(relative(:))/10)+5;
duration=book.time(end)-book.stage1_duration;targets=[0,book.stage1_duration,book.stage1_duration+.33*duration,book.stage1_duration+.67*duration,book.time(end)];
for panel=1:numel(targets)
 ax=axes(fig,'Position',[.09,.82-.18*(panel-1),.88,.12]);hold(ax,'on');axis(ax,'equal');axis(ax,[xmin,xmax,1.875,13.125]);patch(ax,[xmin,xmax,xmax,xmin],[1.875,1.875,13.125,13.125],[.95,.95,.95],'EdgeColor','none');
 for y=[1.875,13.125],plot(ax,[xmin,xmax],[y,y],'k-','LineWidth',.7);end
 for y=[5.625,9.375],plot(ax,[xmin,xmax],[y,y],'--','Color',[.65,.65,.65]);end
 [~,k]=min(abs(book.time-targets(panel)));
 for i=1:size(book.state,3)
  a=find(book.time>=max(book.stage1_duration,book.time(k)-1.5),1);b=find(book.time<=book.time(k)+1.5,1,'last');
  if book.time(k)>=book.stage1_duration
   plot(ax,squeeze(book.state(1,a:k,i))-10*book.time(k),squeeze(book.state(2,a:k,i)),'-','Color',colors(i,:),'LineWidth',1.2);
   plot(ax,squeeze(book.state(1,k:b,i))-10*book.time(k),squeeze(book.state(2,k:b,i)),'--','Color',colors(i,:),'LineWidth',1.1);
  end
 end
 for i=1:size(book.state,3),q=book.state(:,k,i);q(1)=q(1)-10*book.time(k);BookDrawVehicle(ax,q,book.geometry(i),colors(i,:),i,9);end
 text(ax,.5,-.38,sprintf('(%c) t = %.2f s',96+panel,book.time(k)),'Units','normalized','HorizontalAlignment','center','FontSize',16);ylabel(ax,'y (m)');yticks(ax,[3.75,7.5,11.25]);xticks(ax,ceil(xmin/20)*20:20:floor(xmax/20)*20);box(ax,'off');
end
xl=xlabel(ax,'Longitudinal position in a frame traveling at 10 m/s (m)');set(xl,'Units','normalized','Position',[.5,-.75,0]);BookPlotStyle(fig);set(findall(fig,'Type','axes'),'FontSize',13);
exportgraphics(fig,fullfile(out,'lane_change_snapshots.pdf'),'ContentType','vector');exportgraphics(fig,fullfile(out,'lane_change_snapshots.png'),'Resolution',300);close(fig);
end
