function summary=run_experiments(caseNumbers,methods,nodes,budget)
if nargin<1,caseNumbers=1:8;end
if nargin<2,methods={'asco','full','progressive','linearized'};end
if nargin<3,nodes=81;end
if nargin<4,budget=240;end
setup_solver();root=fileparts(mfilename('fullpath'));out=fullfile(root,'output');if ~isfolder(out),mkdir(out);end
summary=struct([]);
for number=caseNumbers
 task=make_case(number);cache=fullfile(out,sprintf('reference_%02d_%d.mat',number,nodes));
 if isfile(cache),data=load(cache);reference=data.reference;else,reference=prepare_reference(task,nodes);save(cache,'reference','task');end
 for m=1:numel(methods)
  method=methods{m};destination=fullfile(out,sprintf('case_%02d_%s.mat',number,method));
  clear solve_reduced
  result=plan_case(task,reference,method,budget);save(destination,'result','reference','-v7');
  item=rmfield(result,{'state','control','task'});summary=[summary,item];
  file=fopen(fullfile(out,sprintf('case_%02d_%s.json',number,method)),'w');fprintf(file,'%s',jsonencode(item));fclose(file);
  fprintf('CASE %d %s: success=%d, %.2f s, T=%.3f, clearance bound=%.4f\n',number,method,result.success,result.wall_seconds,result.horizon,result.validation.interval_clearance_bound);
 end
end
file=fopen(fullfile(out,'experiment_report.json'),'w');fprintf(file,'%s',jsonencode(summary));fclose(file);
end
