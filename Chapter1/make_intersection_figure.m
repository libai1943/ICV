function make_intersection_figure()
root=fileparts(mfilename('fullpath'));addpath(fullfile(root,'common'));BookPlotStyle();colors=BookPalette();out=fullfile(root,'intro_results/intersection_mixed');data=load(fullfile(out,'book_result.mat'));book=data.book;
refined=BookRefinePose(book,20);expanded=refined;
for j=1:size(book.scene.obstacles,1)
 b=book.scene.obstacles(j,:);q=zeros(5,numel(refined.time));q(1,:)=mean(b(1:2));q(2,:)=mean(b(3:4));expanded.state(:,:,end+1)=q;g=expanded.geometry(1);g.LW=0;g.LF=diff(b(1:2))/2;g.LR=g.LF;g.LB=diff(b(3:4));expanded.geometry(end+1)=g;
end
certificate=BookValidatePairs(expanded);assert(certificate.linear_pose_interpolation_certified,'Book:Obstacles','Between-node collision certificate failed.');
ratio=227/118.8;xl=[-48,48];yl=xl/ratio;score=-inf(size(book.time));
for k=1:numel(book.time)
 [visible,incoming,outgoing,central,turning]=trafficView(book,k,xl,yl);
 if sum(outgoing)>=4 && sum(incoming)>=4 && sum(central)>=1
  score(k)=10*sum(visible)+.8*sum(turning)+.25*sum(outgoing)+.2*sum(central);
 end
end
assert(any(isfinite(score)),'Book:Composition','No snapshot contains sufficient incoming and outgoing traffic.');
[~,k]=max(score);
[visible,incoming,outgoing,central,turning]=trafficView(book,k,xl,yl);
colors=colors(mod(0:size(book.state,3)-1,size(colors,1))+1,:);
fig=figure('Color','w','Position',[50,50,1000,round(1000/ratio)]);ax=axes(fig,'Position',[0,0,1,1]);hold(ax,'on');axis(ax,'equal');axis(ax,[xl,yl]);axis(ax,'off');
for j=1:size(book.scene.obstacles,1),b=book.scene.obstacles(j,:);patch(ax,b([1,2,2,1]),b([3,3,4,4]),[.74,.74,.74],'EdgeColor',[.35,.35,.35],'LineWidth',.5);end
for i=find(visible)
 a=find(book.time>=book.time(k)-3,1);b=find(book.time<=book.time(k)+1.4,1,'last');plot(ax,squeeze(book.state(1,a:k,i)),squeeze(book.state(2,a:k,i)),'-','Color',colors(i,:),'LineWidth',1.2);plot(ax,squeeze(book.state(1,k:b,i)),squeeze(book.state(2,k:b,i)),'--','Color',colors(i,:),'LineWidth',.8);
end
for i=find(visible),BookDrawVehicle(ax,book.state(:,k,i),book.geometry(i),colors(i,:),i,12);end
BookPlotStyle(fig);axis(ax,'off');exportgraphics(ax,fullfile(out,'intersection_snapshot.pdf'),'ContentType','vector');exportgraphics(ax,fullfile(out,'intersection_snapshot.png'),'Resolution',400);close(fig);
report=struct('snapshot_time',book.time(k),'vehicle_count',size(book.state,3),'road_width_m',book.scene.parameters.road_width,'vehicle_width_m',1.942,'aspect_ratio',ratio,'method',book.method,'all_vehicle_and_obstacle_interval_bound',certificate.minimum_interval_clearance_bound,'interpolation',certificate.interpolation,'departure_times',book.departure_times);
report.visible_vehicle_ids=find(visible);
report.visible_vehicle_count=sum(visible);report.incoming_vehicle_ids=find(incoming);report.outgoing_vehicle_ids=find(outgoing);report.central_vehicle_ids=find(central);
report.vehicle_length_m=book.geometry(1).LW+book.geometry(1).LF+book.geometry(1).LR;
report.road_to_vehicle_width_ratio=report.road_width_m/report.vehicle_width_m;
report.view_limits_m=[xl,yl];
report.environment_cover='Two overlapping discs, each circumscribing half the physical rectangle; extra STC margin 0.08 m.';
report.scheduling_check='Oriented-rectangle SAT with 0.4 m scheduled-node separation; final independent interval validation.';
report.maximum_discrete_dynamic_defect=0;
report.minimum_two_disc_obstacle_clearance=inf;
report.assembled_trajectory_check=CheckIntroTraffic(book);
for i=1:report.vehicle_count
 filename=fullfile(out,sprintf('single_%02d.mat',i));singleData=load(filename);
 report.maximum_discrete_dynamic_defect=max(report.maximum_discrete_dynamic_defect,singleData.rr.metrics.max_defect);
 report.minimum_two_disc_obstacle_clearance=min(report.minimum_two_disc_obstacle_clearance,singleData.rr.metrics.minimum_clearance);
 for field={'clearance','pair_clearance'}
  if isfield(singleData.rr.metrics,field{1}),singleData.rr.metrics=rmfield(singleData.rr.metrics,field{1});end
 end
 save(filename,'-struct','singleData');
end
fid=fopen(fullfile(out,'figure_validation.json'),'w');fprintf(fid,'%s',jsonencode(report));fclose(fid);disp(report);
end

function [visible,incoming,outgoing,central,turning]=trafficView(book,k,xl,yl)
q=squeeze(book.state(:,k,:));g=book.geometry(1);
a=(g.LW+g.LF+g.LR)/2;b=g.LB/2;h=q(3,:);
cx=q(1,:)+(a-g.LR)*cos(h);cy=q(2,:)+(a-g.LR)*sin(h);
ex=a*abs(cos(h))+b*abs(sin(h));ey=a*abs(sin(h))+b*abs(cos(h));
visible=cx-ex>xl(1)+.1 & cx+ex<xl(2)-.1 & cy-ey>yl(1)+.1 & cy+ey<yl(2)-.1;
radial=cx.*cos(h)+cy.*sin(h);
central=visible & max(abs(cx),abs(cy))<=8;
incoming=visible & ~central & radial<0;
outgoing=visible & ~central & radial>0 & q(4,:)>.2;
turning=visible & abs(sin(2*h))>.3;
end
