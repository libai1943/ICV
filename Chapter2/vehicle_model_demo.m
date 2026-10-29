function report=vehicle_model_demo(out)
% Six-state jerk-extended bicycle model: [x;y;theta;v;phi;a].
% Inputs are JERK and steering rate; no state clipping is performed.
g=struct('LW',2.8,'LF',.96,'LR',.929,'LB',1.942);
opts=odeset('RelTol',1e-10,'AbsTol',1e-11);[t,z]=ode45(@rhs,linspace(0,8,401),[0;0;0;2;0;0],opts);
JERK=.15*cos(pi*t/4);omega=.03*cos(pi*t/4);
assert(max(abs(z(:,6)-.6/pi*sin(pi*t/4)))<1e-8);
assert(max(abs(z(:,5)-.12/pi*sin(pi*t/4)))<1e-8);
writetable(array2table([t,z,JERK,omega],'VariableNames',{'time_s','x_m','y_m','theta_rad','v_m_s','phi_rad','a_m_s2','JERK_m_s3','omega_rad_s'}),fullfile(out,'bicycle_trajectory.csv'));
report=struct('nodes',numel(t),'passed',true,'reference_point','rear-axle center','state_order','x,y,theta,v,phi,a');
 function dz=rhs(t,q)
  dz=[q(4)*cos(q(3));q(4)*sin(q(3));q(4)*tan(q(5))/g.LW;q(6);.03*cos(pi*t/4);.15*cos(pi*t/4)];
 end
end
