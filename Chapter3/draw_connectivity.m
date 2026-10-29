function draw_connectivity(out)
if ~isfolder(out),mkdir(out);end
f=figure('Color','w','Position',[50,50,1000,410],'Visible','off');blue=[.10,.36,.72];gold=[.91,.65,.08];
for panel=1:2
 ax=axes(f,'Position',[.04+(panel-1)*.5,.04,.42,.92]);hold(ax,'on');axis(ax,'equal');axis(ax,[-1.7,1.7,-2.15,2.25]);axis(ax,'off');
 for x=-1.5:1:1.5,plot(ax,[x,x],[-1.5,1.5],'-','Color',[.82,.82,.82]);plot(ax,[-1.5,1.5],[x,x],'-','Color',[.82,.82,.82]);end
 moves=[1,0;-1,0;0,1;0,-1];if panel==2,moves=[moves;1,1;1,-1;-1,1;-1,-1];end
 for j=1:size(moves,1)
  q=moves(j,:);col=blue;if j>4,col=gold;end
  quiver(ax,0,0,.88*q(1),.88*q(2),0,'Color',col,'LineWidth',1.7,'MaxHeadSize',.25);
  plot(ax,q(1),q(2),'o','Color',col,'MarkerFaceColor',.78+.22*col,'MarkerSize',10,'LineWidth',1.2);
 end
 plot(ax,0,0,'o','MarkerFaceColor',[.3,.3,.3],'MarkerEdgeColor','k','MarkerSize',11);
 names={'(a) Four-connected','(b) Eight-connected'};text(ax,0,2.02,names{panel},'FontName','Times New Roman','FontSize',21,'HorizontalAlignment','center');
 notes={'Edge cost: 1',['Cardinal: 1; diagonal: ',char(8730),'2']};text(ax,0,-1.75,notes{panel},'FontName','Times New Roman','FontSize',18,'HorizontalAlignment','center','Interpreter','tex');
end
exportgraphics(f,fullfile(out,'fig03-02.pdf'),'ContentType','vector');exportgraphics(f,fullfile(out,'fig03-02.png'),'Resolution',220);close(f);
end
