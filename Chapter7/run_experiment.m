function result=run_experiment(count)
if nargin<1,count=50;end
setup_solver();root=fileparts(mfilename('fullpath'));out=fullfile(root,'output');if ~isfolder(out),mkdir(out);end
assigned=readmatrix(fullfile(root,'formation_case04.csv'));assigned=assigned(1:count,:);assigned(:,[1,2,4,5])=.3*assigned(:,[1,2,4,5]);
task.geometry=struct('LW',2.8,'LF',.96,'LR',.929,'LB',1.942);g=task.geometry;task.offset=(g.LW+g.LF-g.LR)/2;task.radius=hypot(g.LW+g.LF+g.LR,g.LB)/2;
task.dt=.5;task.nodes=21;task.commit=10;task.margin=.35;task.maxSpeed=2;task.maxAcceleration=1;task.maxSteering=.6;task.maxSteeringRate=.25;
task.goal=assigned(:,4:5)';task.initial=[assigned(:,1:2)'-task.offset*[cos(assigned(:,3)');sin(assigned(:,3)')];assigned(:,3)';zeros(2,count)];task.source_case=4;task.scale=.3;task.source_indices=1:count;
reference=repmat(reshape([task.initial;zeros(2,count)],7,1,count),1,task.nodes,1);trajectory=zeros(7,0,count);history=struct([]);timer=tic;complete=false;
initialAudit=validate_motion(task,reference);assert(initialAudit.passed,'Book:InitialFormation','The stationary initial formation is not collision-free.');
firstWindow=1;checkpoint=fullfile(out,sprintf('checkpoint_%02d.mat',count));
if isfile(checkpoint),saved=load(checkpoint);assert(isequaln(saved.task,task),'Book:CheckpointMismatch','Checkpoint task or numerical settings differ.');reference=saved.reference;trajectory=saved.trajectory;history=saved.history;firstWindow=saved.window+1;end
for window=firstWindow:60
 for local=1:2
  final=max(vecnorm(squeeze(reference(1:2,end,:))+task.offset*[cos(squeeze(reference(3,end,:)))';sin(squeeze(reference(3,end,:)))']-task.goal))<.5;
  [candidate,diagnostic]=solve_window(task,reference,final);diagnostic.window=window;diagnostic.local=local;
  history=[history,diagnostic];save(fullfile(out,'latest_window.mat'),'task','reference','candidate','diagnostic','history','trajectory');fprintf('WINDOW %d.%d: %s, goal %.4f m, clearance %.4f m, %.1f s\n',window,local,diagnostic.status,diagnostic.maximum_goal_error,diagnostic.clearance_bound,diagnostic.seconds);
  if ~diagnostic.feasible
   save(fullfile(out,'failed_window.mat'),'task','reference','candidate','diagnostic');
   if local>1
    warning('Book:RefinementRejected','Retaining the validated first refinement of this window.');break;
   end
   error('Book:WindowFailure','No validated local solution; previous output retained.');
  end
  reference=candidate;if final && diagnostic.maximum_goal_error<1e-4,complete=true;break;end
 end
 if complete,trajectory=cat(2,trajectory,reference);break;end
 trajectory=cat(2,trajectory,reference(:,1:task.commit,:));
 reference=cat(2,reference(:,task.commit+1:end,:),repmat(reference(:,end,:),1,task.commit,1));reference(6:7,end-task.commit+1:end,:)=0;
 save(checkpoint,'reference','trajectory','history','window','task');
 result=struct('task',task,'trajectory',trajectory,'history',history,'completed',false,'wall_seconds',toc(timer));save(fullfile(out,sprintf('partial_%02d.mat',count)),'result');
end
result=struct('task',task,'trajectory',trajectory,'history',history,'completed',complete,'wall_seconds',toc(timer));result.validation=validate_motion(task,trajectory);
result.success=complete&&result.validation.passed;save(fullfile(out,sprintf('formation_%02d.mat',count)),'result','-v7');
report=rmfield(result,{'trajectory'});fid=fopen(fullfile(out,sprintf('formation_%02d.json',count)),'w');fprintf(fid,'%s',jsonencode(report));fclose(fid);
assert(result.success,'Book:Incomplete','Formation did not meet the final validation criteria.');
end
