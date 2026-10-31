function run_full_budget()
% Repeat the full-NLP baseline with its complete 360-second case budget.
setup_solver();root=fileparts(mfilename('fullpath'));out=fullfile(root,'output/full_budget360');if ~isfolder(out),mkdir(out);end
for number=1:8
 cache=fullfile(root,sprintf('output/reference_%02d_41.mat',number));while ~isfile(cache),pause(10);end
 data=load(cache);reference=data.reference;task=make_case(number);[d,~]=collision_vector(reference.state,task);clock=tic;clear solve_reduced
 [candidate,info]=solve_reduced(task,reference,true(numel(d),1),'exact',360);info.outer_iteration=1;info.lower_threshold=-inf;info.upper_threshold=inf;
 validation=validate_plan(candidate,task);result=struct('method','full','case_id',task.id,'state',candidate.state,'control',candidate.control,'horizon',candidate.horizon,'task',task,'history',info,'wall_seconds',toc(clock),'validation',validation,'success',info.success&&validation.passed);
 save(fullfile(out,sprintf('case_%02d_full.mat',number)),'result','reference','-v7');item=rmfield(result,{'state','control','task'});f=fopen(fullfile(out,sprintf('case_%02d_full.json',number)),'w');fprintf(f,'%s',jsonencode(item));fclose(f);
 fprintf('FULL360 case %d: success=%d, %.1f s, %s, clearance %.4f\n',number,result.success,result.wall_seconds,info.status,validation.interval_clearance_bound);
end
end
