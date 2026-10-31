function report=CheckIntroTraffic(book)
% Independent checks of the complete assembled arrival/turn/departure history.
p=book.scene.parameters;dt=diff(book.time);nv=size(book.state,3);
assert(max(abs(dt-p.dt))<1e-9,'Book:Time','Nonuniform time grid.');
report=struct('vehicle_count',nv,'nodes',numel(book.time), ...
    'maximum_euler_residual',0,'maximum_bound_violation',0, ...
    'minimum_speed_m_s',inf,'maximum_speed_m_s',0, ...
    'maximum_acceleration_m_s2',0,'maximum_steering_rad',0, ...
    'maximum_steering_rate_rad_s',0);
for i=1:nv
    q=book.state(:,:,i);u=book.control(:,:,i);left=q(:,1:end-1);
    assert(all(isfinite(q(:))) && all(isfinite(u(:))),'Book:Finite','Nonfinite trajectory.');
    rate=[left(4,:).*cos(left(3,:));left(4,:).*sin(left(3,:)); ...
        left(4,:).*tan(left(5,:))/book.geometry(i).LW;u(:,1:end-1)];
    defect=diff(q,1,2)-rate.*dt;
    report.maximum_euler_residual=max(report.maximum_euler_residual,max(abs(defect(:))));
    violation=[p.vmin-q(4,:),q(4,:)-p.vmax,abs(q(5,:))-p.phimax, ...
        abs(u(1,:))-p.amax,abs(u(2,:))-p.omegamax];
    report.maximum_bound_violation=max(report.maximum_bound_violation,max(violation));
    report.minimum_speed_m_s=min(report.minimum_speed_m_s,min(q(4,:)));
    report.maximum_speed_m_s=max(report.maximum_speed_m_s,max(q(4,:)));
    report.maximum_acceleration_m_s2=max(report.maximum_acceleration_m_s2,max(abs(u(1,:))));
    report.maximum_steering_rad=max(report.maximum_steering_rad,max(abs(q(5,:))));
    report.maximum_steering_rate_rad_s=max(report.maximum_steering_rate_rad_s,max(abs(u(2,:))));
end
report.scope='All 24 trajectories, including advancing queues, local maneuvers, seams, and continuing outbound traffic; direct Euler residual, not an exact continuous-time integration claim.';
report.passed=report.maximum_euler_residual<1e-6 && report.maximum_bound_violation<1e-6;
assert(report.passed,'Book:TrafficDynamics','Assembled trajectory check failed.');
end
