function export_results()
root=fileparts(mfilename('fullpath'));out=fullfile(root,'output');names={'04','08','12','08_recovery'};summary={};
for n=1:numel(names)
 file=fullfile(out,['case_',names{n},'.mat']);if ~isfile(file),continue;end
 d=load(file);r=d.result;q=r.trajectory;nodes=size(q,2);dt=r.scene.horizon/(nodes-1);r.continuous_control_audit=audit_control_bounds(q(1:6,:,:),q(7:8,1:end-1,:),r.scene.horizon,r.scene,true);assert(~r.success||r.continuous_control_audit.passed,'Continuous control bounds failed.');
 rows=[];for car=1:size(q,3),rows=[rows;((0:nodes-1)*dt)',repmat(car,nodes,1),q(1:8,:,car)'];end
 headers={'time_s','vehicle_id','x_m','y_m','heading_rad','velocity_mps','steering_rad','acceleration_mps2','jerk_mps3','steering_rate_radps'};writetable(array2table(rows,'VariableNames',headers),fullfile(out,['trajectory_',names{n},'.csv']));
 item=struct('vehicles',size(q,3),'experiment',names{n},'success',r.success,'wall_seconds',r.wall_seconds,'history',r.history,'validation',rmfield(r.validation,'clearance'),'continuous_control_audit',r.continuous_control_audit,'initialization',r.initialization);summary{end+1}=item;
 f=fopen(fullfile(out,['case_',names{n},'.json']),'w');fprintf(f,'%s',jsonencode(item));fclose(f);
end
f=fopen(fullfile(out,'experiment_report.json'),'w');fprintf(f,'%s',jsonencode(summary));fclose(f);
end
