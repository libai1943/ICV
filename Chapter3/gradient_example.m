function result=gradient_example(out)
% Local-solution sensitivity using the book-wide IPOPT/MA57 configuration.
setup_solver();import casadi.*
x=SX.sym('x');f=tanh(x)+.2*x*exp(-.1*x^2)+.3*sin(2.5*x);
options=struct('print_time',false);options.ipopt=struct('print_level',0,'sb','yes','tol',1e-9,'max_iter',500,'bound_relax_factor',0,'linear_solver','ma57','hsllib',getenv('IPOPT_HSL_LIBRARY'));
solver=nlpsol('local_example','ipopt',struct('x',x,'f',f),options);derivative=Function('derivative',{x},{jacobian(f,x)});
starts=[-6,-3.2,-1,-.2,.05,.8,1.8,2.3,3.14,3.5,4.2,5.5];result=zeros(numel(starts),4);statuses=cell(numel(starts),1);
for j=1:numel(starts)
 solution=solver('x0',starts(j),'lbx',-8,'ubx',8);stats=solver.stats();terminal=full(solution.x);slope=full(derivative(terminal));kkt=abs(slope)<1e-7||(abs(terminal+8)<1e-6&&slope>=-1e-7)||(abs(terminal-8)<1e-6&&slope<=1e-7);assert(stats.success&&kkt&&terminal>=-8&&terminal<=8);
 result(j,:)=[starts(j),terminal,full(solution.f),stats.iter_count];statuses{j}=stats.return_status;
end
writetable(array2table(result,'VariableNames',{'initial_x','terminal_x','cost','iterations'}),fullfile(out,'gradient_results.csv'));
fid=fopen(fullfile(out,'optimizer_status.json'),'w');fprintf(fid,'%s',jsonencode(struct('optimizer','IPOPT','linear_solver','MA57','statuses',{statuses})));fclose(fid);
figure('Visible','off','Color','w','Position',[50,50,1100,460]);xx=linspace(-8,8,4001);yy=tanh(xx)+.2*xx.*exp(-.1*xx.^2)+.3*sin(2.5*xx);plot(xx,yy,'k-','LineWidth',1.3);hold on;
for j=1:size(result,1)
 x0=starts(j);f0=tanh(x0)+.2*x0*exp(-.1*x0^2)+.3*sin(2.5*x0);
 plot([x0,result(j,2)],[f0,result(j,3)],':','Color',[.6,.6,.6]);plot(x0,f0,'o','MarkerSize',4,'Color',[.25,.45,.65]);
end
plot(result(:,2),result(:,3),'s','MarkerSize',6,'MarkerFaceColor',[.84,.46,.22],'MarkerEdgeColor',[.55,.25,.1]);xlabel('{\it x}');ylabel('{\it f}({\it x})');grid on;set(gca,'FontName','Times New Roman','FontSize',15);set(findall(gcf,'Type','text'),'FontName','Times New Roman');
exportgraphics(gcf,fullfile(out,'fig03-01.pdf'),'ContentType','vector');exportgraphics(gcf,fullfile(out,'fig03-01.png'),'Resolution',180);close(gcf);
end
