function BookDrawVehicle(ax,q,g,color,id,labelSize)
%BOOKDRAWVEHICLE Actual rectangle and a central forward-pointing triangle.
if nargin<5,id=[];end
if nargin<6,labelSize=11;end
L=g.LW+g.LF+g.LR;rot=[cos(q(3)),-sin(q(3));sin(q(3)),cos(q(3))];
body=[-g.LR,L-g.LR,L-g.LR,-g.LR;-g.LB/2,-g.LB/2,g.LB/2,g.LB/2];body=rot*body+q(1:2);
patch(ax,body(1,:),body(2,:),.76+.24*color,'EdgeColor',color,'LineWidth',.9);
triangle=[-g.LR+.29*L,-g.LR+.29*L,-g.LR+.64*L;-g.LB/2,g.LB/2,0];triangle=rot*triangle+q(1:2);
patch(ax,triangle(1,:),triangle(2,:),.55+.45*color,'EdgeColor',color,'LineWidth',.7,'FaceAlpha',1);
% Four physical tires; front-wheel orientation is the computed steering angle.
for axle=[0,g.LW]
 for side=[-1,1]
  center=q(1:2)+rot*[axle;side*g.LB*.38];angle=q(3);if axle>0&&numel(q)>=5,angle=angle+q(5);end
  tire=[cos(angle),-sin(angle);sin(angle),cos(angle)]*[-.28,.28,.28,-.28;-.09,-.09,.09,.09]+center;
  patch(ax,tire(1,:),tire(2,:),[.2,.22,.24],'EdgeColor',[.2,.22,.24],'LineWidth',.4);
 end
end
if ~isempty(id)
 center=q(1:2)+(-g.LR+.80*L)*[cos(q(3));sin(q(3))];
 text(ax,center(1),center(2),num2str(id),'HorizontalAlignment','center','VerticalAlignment','middle','FontSize',labelSize,'Color','k','BackgroundColor','none');
end
end
