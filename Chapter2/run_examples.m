function report=run_examples()
root=fileparts(mfilename('fullpath'));addpath(root);report=draw_figures(fullfile(root,'output'));report.bicycle=vehicle_model_demo(fullfile(root,'output'));report.figures{end+1}='fig02-10';f=fopen(fullfile(root,'output','validation.json'),'w');fprintf(f,'%s',jsonencode(report));fclose(f);disp(report);
end
