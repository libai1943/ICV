function result=run_example()
% Solve the fixed ten-ICV ASCO demonstration from its common initialization.
setup_solver();root=fileparts(mfilename('fullpath'));out=fullfile(root,'output');if ~isfolder(out),mkdir(out);end
task=make_case();reference=prepare_reference(task,81);
result=plan_case(task,reference,480);
assert(result.success,'The candidate did not pass the complete trajectory checks.');
save(fullfile(out,'case_01_asco.mat'),'result','reference','-v7');
export_results();make_figures();
fprintf('Validated ASCO example: ten ICVs, %.2f s maneuver.\n',result.horizon);
end
