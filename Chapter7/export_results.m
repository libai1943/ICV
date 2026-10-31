function export_results()
root=fileparts(mfilename('fullpath'));out=fullfile(root,'output');d=load(fullfile(out,'formation_50.mat'));r=d.result;assert(r.success);q=r.trajectory;nodes=size(q,2);rows=[];
for car=1:size(q,3),rows=[rows;((0:nodes-1)*r.task.dt)',repmat(car,nodes,1),q(:,:,car)'];end
writetable(array2table(rows,'VariableNames',{'time_s','vehicle_id','x_m','y_m','heading_rad','velocity_mps','steering_rad','acceleration_mps2','steering_rate_radps'}),fullfile(out,'trajectory_50.csv'));
report=rmfield(r,'trajectory');report.discretization='first-order explicit forward Euler';report.optimizer='IPOPT';report.linear_solver='MA57';report.maneuver_seconds=(nodes-1)*r.task.dt;report.summed_local_solve_seconds=sum([r.history.seconds]);report.maximum_goal_error=max(vecnorm(squeeze(q(1:2,end,:))+r.task.offset*[cos(reshape(q(3,end,:),1,[]));sin(reshape(q(3,end,:),1,[]))]-r.task.goal));
fid=fopen(fullfile(out,'experiment_report.json'),'w');fprintf(fid,'%s',jsonencode(report));fclose(fid);
end
