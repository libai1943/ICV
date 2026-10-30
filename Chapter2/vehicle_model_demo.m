function report=vehicle_model_demo(out)
% Six-state jerk-extended bicycle model: [x;y;theta;v;phi;a].
% Inputs are jerk and steering rate; no state clipping is performed.
g=struct('LW',2.8,'LF',.96,'LR',.929,'LB',1.942);
opts=odeset('RelTol',1e-10,'AbsTol',1e-11);[t,z]=ode45(@rhs,linspace(0,8,401),[0;0;0;2;0;0],opts);
jerk=.15*cos(pi*t/4);omega=.03*cos(pi*t/4);
assert(max(abs(z(:,6)-.6/pi*sin(pi*t/4)))<1e-8);
assert(max(abs(z(:,5)-.12/pi*sin(pi*t/4)))<1e-8);
writetable(array2table([t,z,jerk,omega],'VariableNames',{'time_s','x_m','y_m','theta_rad','v_m_s','phi_rad','a_m_s2','jerk_m_s3','omega_rad_s'}),fullfile(out,'bicycle_trajectory.csv'));

f=figure('Color','w','Position',[50,50,1000,430],'Visible','off');
ax=axes(f,'Position',[.09,.24,.38,.70]);plot(ax,z(:,1),z(:,2),'Color',[.10,.36,.72],'LineWidth',1.7);hold(ax,'on');
for k=[1,101,201,301,401],plot(ax,z(k,1),z(k,2),'ko','MarkerFaceColor','w');text(ax,z(k,1),z(k,2)+.04,sprintf('%.0f s',t(k)),'HorizontalAlignment','center','FontSize',15);end
xlabel(ax,'x (m)');ylabel(ax,'y (m)');grid(ax,'on');box(ax,'off');ylim(ax,[-.1,max(z(:,2))+.18]);set(ax,'FontName','Times New Roman','FontSize',16);
text(ax,.5,-.24,'(a) Rear-axle trajectory','Units','normalized','HorizontalAlignment','center','FontSize',20);
ax=axes(f,'Position',[.60,.24,.35,.70]);plot(ax,t,z(:,6),'-','Color',[.10,.36,.72],'LineWidth',1.6);hold(ax,'on');plot(ax,t,jerk,'--','Color',[.89,.56,.10],'LineWidth',1.6);
xlabel(ax,'t (s)');ylabel(ax,'Acceleration / jerk');legend(ax,{'a (m/s^2)','jerk (m/s^3)'},'Box','off','Location','southwest','FontSize',14);grid(ax,'on');box(ax,'off');set(ax,'FontName','Times New Roman','FontSize',16);
text(ax,.5,-.24,'(b) State and applied control','Units','normalized','HorizontalAlignment','center','FontSize',20);
exportgraphics(f,fullfile(out,'fig02-10.pdf'),'ContentType','vector');exportgraphics(f,fullfile(out,'fig02-10.png'),'Resolution',220);close(f);
report=struct('nodes',numel(t),'passed',true,'reference_point','rear-axle center','state_order','x,y,theta,v,phi,a','max_acceleration_error',max(abs(z(:,6)-.6/pi*sin(pi*t/4))),'max_steering_error',max(abs(z(:,5)-.12/pi*sin(pi*t/4))));
 function dz=rhs(t,q)
  dz=[q(4)*cos(q(3));q(4)*sin(q(3));q(4)*tan(q(5))/g.LW;q(6);.03*cos(pi*t/4);.15*cos(pi*t/4)];
 end
end
