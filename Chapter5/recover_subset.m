function recover_subset()
% A separately reported initialization experiment, not a replacement of failure.
setup_solver();root=fileparts(mfilename('fullpath'));out=fullfile(root,'output');data=load(fullfile(out,'case_12.mat'));assert(data.result.success);
scene=make_scenario(8);parent=data.result;assert(max(abs(parent.scene.start(:,1:8)-scene.start),[],'all')<1e-10);assert(max(abs(parent.scene.goal(:,1:8)-scene.goal),[],'all')<1e-10);
reference=parent.trajectory(:,:,1:8);result=optimize_trajectories(scene,reference);result.initialization=struct('source','restriction of validated twelve-vehicle solution','independent_size_trial',false);
save(fullfile(out,'case_08_recovery.mat'),'result','reference','-v7');item=struct('vehicles',8,'wall_seconds',result.wall_seconds,'success',result.success,'history',result.history,'validation',rmfield(result.validation,'clearance'),'initialization',result.initialization);file=fopen(fullfile(out,'case_08_recovery.json'),'w');fprintf(file,'%s',jsonencode(item));fclose(file);fprintf('SUBSET RECOVERY: success=%d\n',result.success);
end
