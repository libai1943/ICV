function export_results()
root=fileparts(mfilename('fullpath'));out=fullfile(root,'output');methods={'asco'};summary={};
for n=1,for m=1
 file=fullfile(out,sprintf('case_%02d_%s.mat',n,methods{m}));d=load(file);r=finalize_plan(d.result);result=r;reference=d.reference;save(file,'result','reference','-v7');q=r.state;nodes=size(q,2);dt=r.horizon/(nodes-1);r.continuous_control_audit=audit_control_bounds(q,r.control,r.horizon,r.task,false);assert(~r.success||r.continuous_control_audit.passed,'Continuous control bounds failed.');
 rows=[];for car=1:size(q,3),rows=[rows;((0:nodes-1)*dt)',repmat(car,nodes,1),q(:,:,car)',[r.control(:,:,car),zeros(2,1)]'];end
 headers={'time_s','vehicle_id','x_m','y_m','heading_rad','velocity_mps','steering_rad','acceleration_mps2','jerk_mps3','steering_rate_radps'};writetable(array2table(rows,'VariableNames',headers),fullfile(out,sprintf('trajectory_%02d_%s.csv',n,methods{m})));
 item=rmfield(r,{'state','control','task'});summary{end+1}=item;f=fopen(fullfile(out,sprintf('case_%02d_%s.json',n,methods{m})),'w');fprintf(f,'%s',jsonencode(item));fclose(f);
end,end
f=fopen(fullfile(out,'experiment_report.json'),'w');fprintf(f,'%s',jsonencode(summary));fclose(f);
end
