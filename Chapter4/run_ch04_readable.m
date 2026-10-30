function book = run_ch04_readable(config)
if nargin<1,config=struct();end
%RUN_CH04_READABLE Editable MATLAB/CasADi implementation of the two-stage method.
% Method: Li et al., IEEE TIV 2018, doi:10.1109/TIV.2018.2843159.
% Input generation, dimensions, bounds, priority ordering and translation step
% follow libai1943/MultiCavLaneChange (9831df03d1f78ff913fc686ff8b9c49c480feacc).
% All model equations are visible here. No .p function or AMPL is called.
% The book's five-state model is transcribed with the trapezoidal rule.
% Rectangular-body separation is certified for the declared pose interpolant.
root=fileparts(mfilename('fullpath')); oldPath=path;
cleanup=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(fullfile(root,'common'));
package=getenv('CASADI_MATLAB_PATH');
if ~isempty(package),assert(isfolder(package),'Book:CasADiPath','CASADI_MATLAB_PATH must name the CasADi package folder.');addpath(package);end
assert(exist('casadiMEX','file')==3,'Book:CasADi','CasADi MATLAB package is required.');
import casadi.*
seed=1943;if isfield(config,'seed'),seed=config.seed;end
rng(seed,'twister');
g=struct('LW',2.8,'LF',.96,'LR',.929,'LB',1.942);
p=struct('N',10,'lanes',3,'lane_width',3.75,'v0',10,'vmax',12,'amax',.5, ...
    'phimax',.6,'omegamax',.25,'translation_step',.5,'clearance',.05, ...
    'road_lower',1.875,'road_upper',13.125,'K1',300,'K2',100, ...
    'acceleration_weight',.01,'steering_rate_weight',.01,'jerk_limit',inf);
if isfield(config,'vehicle_count'),p.N=config.vehicle_count;end
if isfield(config,'jerk_limit'),p.jerk_limit=config.jerk_limit;end
if isfield(config,'K1'),p.K1=config.K1;end
if isfield(config,'K2'),p.K2=config.K2;end
p.seed=seed;
task=makeTask(p,g);timer=tic;
[stage2,solver2]=solveLaneChanges(task,p,g);
fprintf('Readable chapter 4: lane-change stage %.6f s, %d solver iterations.\n',stage2.time(end),solver2.iterations);
[sparse,translations]=makeSparse(stage2,task,p,g);
[stage1,solver1]=solveFormation(task,sparse.state(1,1,:),p,g);
shift=squeeze(stage1.state(1,end,:)-sparse.state(1,1,:));
assert(max(shift)-min(shift)<1e-5,'Book:Formation','Stage junction must use a common longitudinal translation.');
for i=1:p.N,sparse.state(1,:,i)=sparse.state(1,:,i)+shift(i);end
assert(max(abs(stage1.state(:,end,:)-sparse.state(:,1,:)),[],'all')<1e-5,'Book:Junction','Stage states do not match.');
book.schema_version=1;book.chapter=4;book.time=[stage1.time,stage1.time(end)+sparse.time(2:end)];
book.state=cat(2,stage1.state,sparse.state(:,2:end,:));
book.control=cat(2,stage1.control,sparse.control(:,2:end,:));
book.geometry=repmat(g,1,p.N);book.state_names={'x','y','theta','v','phi'};book.control_names={'a','omega'};
book.state_units={'m','m','rad','m/s','rad'};book.control_units={'m/s^2','rad/s'};book.reference_point='rear-axle center';
book.stage1_duration=stage1.time(end);book.planning_seconds=toc(timer);book.task=task;book.parameters=p;
book.source_commit='9831df03d1f78ff913fc686ff8b9c49c480feacc';
book.implementation='Editable MATLAB/CasADi two-stage method; trapezoidal transcription, rectangle clearance screening, time plus quadratic control cost.';
book.time_provenance='Native stage node times concatenated once at the common junction.';
book.matlab_version=version;book.solver=struct('stage1',solver1,'stage2',solver2);
book.translations=translations;book.validation=BookValidatePairs(book);
book.interpolation_validation=BookValidatePairs(BookRefinePose(book,20));
book.interpolation_validation.refinement_factor=20;
book.model_validation=validateModel(book,stage1,sparse,task,p,g);
assert(book.interpolation_validation.linear_pose_interpolation_certified,'Book:Collision','Interpolated vehicle rectangles are not certified separated.');
assert(book.model_validation.maximum_trapezoidal_rate_residual<1e-5,'Book:Kinematics','Discrete kinematic residual exceeds tolerance.');
assert(book.model_validation.maximum_boundary_error<1e-5,'Book:Boundary','Boundary conditions are not satisfied.');
assert(book.model_validation.bounds_satisfied,'Book:Bounds','State or control bounds violated.');
assert(book.model_validation.minimum_sampled_road_clearance>0,'Book:Road','Vehicle footprint leaves the road.');
assert(book.model_validation.minimum_interval_road_clearance_bound>0,'Book:Road','Interpolated road containment not certified.');
assert(book.model_validation.maximum_integrated_position_error<.02,'Book:Integration','Forward-integrated position differs by more than 2 cm.');
out=fullfile(root,'ch04_lane_change/results_readable');if isfield(config,'output_directory'),out=config.output_directory;end
if ~isfolder(out),mkdir(out);end
BookExportTables(book,out);save(fullfile(out,'book_result.mat'),'book','stage1','sparse','stage2','-v7.3');
report=struct('sampled',book.validation,'interpolation',book.interpolation_validation,'model',book.model_validation,'solver',book.solver);
fid=fopen(fullfile(out,'book_validation.json'),'w');assert(fid>=0);closeFile=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(report));
fprintf('Readable chapter 4 complete: %d vehicles, %.6f s maneuver, %.3f s planning.\n',p.N,book.time(end),book.planning_seconds);
disp(book.model_validation);
disp(rmfield(book.interpolation_validation,{'sampled_gap','interval_bound'}));
end

function task=makeTask(p,g)
% Same random draws as the public repository, without filesystem side effects.
x=randn(1,p.lanes);task=zeros(p.N,3);L=g.LW+g.LF+g.LR;
for i=1:p.N
 a=ceil(rand*p.lanes);b=ceil(rand*p.lanes);xi=x(a)-L-rand*5;x(a)=xi;
 task(i,:)=[xi,a*p.lane_width,b*p.lane_width];
end
end

function [stage,info]=solveLaneChanges(task,p,g)
import casadi.*
K=p.K2;op=Opti();T=op.variable();Z=op.variable(5*p.N,K);U=op.variable(2*p.N,K);h=T/(K-1);
bound(op,.5,T,30);cost=T;
T0=8;t=linspace(0,T0,K);s=t/T0;blend=10*s.^3-15*s.^4+6*s.^5;
dblend=(30*s.^2-60*s.^3+30*s.^4)/T0;
for i=1:p.N
 iz=(i-1)*5+(1:5);iu=(i-1)*2+(1:2);z=Z(iz,:);u=U(iu,:);
 f=kinematics(z,u,g);
 op.subject_to(z(:,2:end)-z(:,1:end-1)==h/2*(f(:,1:end-1)+f(:,2:end)));
 op.subject_to(z(:,1)==[0;task(i,2);0;p.v0;0]);
 op.subject_to(z([2,3,4,5],end)==[task(i,3);0;p.v0;0]);
 op.subject_to(u(:,1)==0);op.subject_to(u(:,end)==0);
 bound(op,0,z(4,:),p.vmax);bound(op,-pi/2,z(3,:),pi/2);
 bound(op,-p.phimax,z(5,:),p.phimax);
 bound(op,-p.amax,u(1,:),p.amax);bound(op,-p.omegamax,u(2,:),p.omegamax);
 if isfinite(p.jerk_limit),bound(op,-p.jerk_limit,diff(u(1,:))/h,p.jerk_limit);end
 for longitudinal=[-g.LR,g.LW+g.LF]
  for lateral=[-g.LB/2,g.LB/2]
   yc=z(2,:)+longitudinal*sin(z(3,:))+lateral*cos(z(3,:));
   bound(op,p.road_lower+p.clearance,yc,p.road_upper-p.clearance);
  end
 end
 weights=[.5,ones(1,K-2),.5];
 cost=cost+h*(p.acceleration_weight*sum2(weights.*u(1,:).^2)+p.steering_rate_weight*sum2(weights.*u(2,:).^2));
 dy=task(i,3)-task(i,2);ydot=dy*dblend;theta=atan2(ydot,p.v0);v=hypot(p.v0,ydot);
 phi=atan(g.LW*gradient(theta,t)./v);a=gradient(v,t);omega=gradient(phi,t);
 op.set_initial(z,[p.v0*t;task(i,2)+dy*blend;theta;v;phi]);op.set_initial(u,[a;omega]);
end
op.set_initial(T,T0);op.minimize(cost);configureSolver(op);sol=op.solve();
stage.time=linspace(0,sol.value(T),K);stage.state=zeros(5,K,p.N);stage.control=zeros(2,K,p.N);
for i=1:p.N
 stage.state(:,:,i)=sol.value(Z((i-1)*5+(1:5),:));stage.state(1,:,i)=stage.state(1,:,i)+task(i,1);
 stage.control(:,:,i)=sol.value(U((i-1)*2+(1:2),:));
end
stats=sol.stats();info=struct('status',stats.return_status,'iterations',stats.iter_count,'cost',sol.value(cost),'transcription','trapezoidal');
end

function f=kinematics(z,u,g)
f=[z(4,:).*cos(z(3,:));z(4,:).*sin(z(3,:));z(4,:).*tan(z(5,:))/g.LW;u(1,:);u(2,:)];
end

function [stage,translations]=makeSparse(stage,task,p,g)
% Same descending-x priority and same-lane propagation as ProduceDelayList.m.
% Replace the protected geometric test with explicit rectangle SAT and an
% interval displacement bound. This avoids relying on vertex-only tests.
[~,order]=sort(task(:,1),'descend');translations=zeros(p.N,1);
for rank=2:p.N
 id=order(rank);attempts=0;
 while true
  ids=[order(1:rank-1);id];test.time=stage.time;test.state=stage.state(:,:,ids);test.geometry=repmat(g,1,numel(ids));
  check=BookValidatePairs(BookRefinePose(test,8));
  if check.minimum_interval_clearance_bound>=p.clearance,break;end
  attempts=attempts+1;assert(attempts<=2000,'Book:TranslationLimit','Sparse-formation construction exceeded its translation budget.');
  rest=order(rank:end);move=rest(task(rest,2)==task(id,2));
  stage.state(1,:,move)=stage.state(1,:,move)-p.translation_step;translations(move)=translations(move)-p.translation_step;
 end
end
fprintf('Sparse formation: largest backward translation %.3f m.\n',max(abs(translations)));
end

function [stage,info]=solveFormation(task,sparseX,p,g)
import casadi.*
K=p.K1;N=p.N;target=squeeze(sparseX);op=Opti();T=op.variable();X=op.variable(N,K);V=op.variable(N,K);A=op.variable(N,K);h=T/(K-1);
bound(op,.5,T,120);op.subject_to(X(:,1)==task(:,1));
op.subject_to(V(:,1)==p.v0);op.subject_to(V(:,end)==p.v0);op.subject_to(A(:,1)==0);op.subject_to(A(:,end)==0);
bound(op,0,V,p.vmax);bound(op,-p.amax,A,p.amax);
if isfinite(p.jerk_limit),bound(op,-p.jerk_limit,diff(A,1,2)/h,p.jerk_limit);end
op.subject_to(X(:,2:end)-X(:,1:end-1)==h/2*(V(:,1:end-1)+V(:,2:end)));
op.subject_to(V(:,2:end)-V(:,1:end-1)==h/2*(A(:,1:end-1)+A(:,2:end)));
op.subject_to(X(2:end,end)-X(1,end)==target(2:end)-target(1));
L=g.LW+g.LF+g.LR;
for lane=1:p.lanes
 ids=find(task(:,2)==lane*p.lane_width);[~,ix]=sort(task(ids,1),'descend');ids=ids(ix);
 for j=2:numel(ids),op.subject_to(X(ids(j-1),:)-X(ids(j),:)>=L+p.clearance);end
end
weights=[.5,ones(1,K-2),.5];cost=T+p.acceleration_weight*h*sum1(sum2(A.^2.*repmat(weights,N,1)));op.minimize(cost);
delta=target-task(:,1);delta=delta-(min(delta)+max(delta))/2;
T0=max(8,sqrt(max(abs(delta))*6/p.amax)*1.15);s=linspace(0,1,K);
blend=10*s.^3-15*s.^4+6*s.^5;dblend=(30*s.^2-60*s.^3+30*s.^4)/T0;ddblend=(60*s-180*s.^2+120*s.^3)/T0^2;
op.set_initial(T,T0);op.set_initial(X,task(:,1)+p.v0*T0*s+delta*blend);op.set_initial(V,p.v0+delta*dblend);op.set_initial(A,delta*ddblend);
configureSolver(op);sol=op.solve();stage.time=linspace(0,sol.value(T),K);stage.state=zeros(5,K,N);stage.control=zeros(2,K,N);
xx=sol.value(X);vv=sol.value(V);aa=sol.value(A);
for i=1:N,stage.state(:,:,i)=[xx(i,:);repmat(task(i,2),1,K);zeros(1,K);vv(i,:);zeros(1,K)];stage.control(1,:,i)=aa(i,:);end
stats=sol.stats();info=struct('status',stats.return_status,'iterations',stats.iter_count,'cost',sol.value(cost),'transcription','trapezoidal');
end

function configureSolver(op)
options=struct('expand',true,'print_time',false);options.ipopt=struct('print_level',0,'tol',1e-8,'constr_viol_tol',1e-8,'max_iter',1500,'linear_solver','mumps','sb','yes');op.solver('ipopt',options);
end

function m=validateModel(book,s1,s2,task,p,g)
res=0;
for stage={s1,s2}
 a=stage{1};h=diff(a.time);
 for i=1:p.N
  z=a.state(:,:,i);u=a.control(:,:,i);f=kinematics(z,u,g);
  res=max(res,max(abs(diff(z,1,2)./h-.5*(f(:,1:end-1)+f(:,2:end))),[],'all'));
 end
end
m.maximum_trapezoidal_rate_residual=res;
m.maximum_jerk=0;
for stage={s1,s2}
 aa=stage{1};jerk=diff(aa.control(1,:,:),1,2)./diff(aa.time);m.maximum_jerk=max(m.maximum_jerk,max(abs(jerk),[],'all'));
end
assert(m.maximum_jerk<=p.jerk_limit+1e-7,'Book:Jerk','Jerk bound violated.');
z0=squeeze(book.state(:,1,:));zf=squeeze(book.state(:,end,:));
errors=[z0-[task(:,1:2)';zeros(1,p.N);p.v0*ones(1,p.N);zeros(1,p.N)];zf([2,3,4,5],:)-[task(:,3)';zeros(1,p.N);p.v0*ones(1,p.N);zeros(1,p.N)]];
m.maximum_boundary_error=max(abs(errors),[],'all');
m.max_abs_speed=max(abs(book.state(4,:,:)),[],'all');m.max_abs_steering_angle=max(abs(book.state(5,:,:)),[],'all');
m.max_abs_acceleration=max(abs(book.control(1,:,:)),[],'all');m.max_abs_steering_rate=max(abs(book.control(2,:,:)),[],'all');
m.bounds_satisfied=min(book.state(4,:,:),[],'all')>=-1e-7 && m.max_abs_speed<=p.vmax+1e-7 && m.max_abs_steering_angle<=p.phimax+1e-7 && m.max_abs_acceleration<=p.amax+1e-7 && m.max_abs_steering_rate<=p.omegamax+1e-7;
fine=BookRefinePose(book,20);clearance=inf(1,numel(fine.time));bound=inf(1,numel(fine.time)-1);
for i=1:p.N
 z=fine.state(:,:,i);c=inf(1,numel(fine.time));
 for longitudinal=[-g.LR,g.LW+g.LF]
  for lateral=[-g.LB/2,g.LB/2]
   yc=z(2,:)+longitudinal*sin(z(3,:))+lateral*cos(z(3,:));c=min(c,min(yc-p.road_lower,p.road_upper-yc));
  end
 end
 radius=hypot(max(g.LR,g.LW+g.LF),g.LB/2);motion=abs(diff(z(2,:)))+radius*abs(diff(z(3,:)));
 clearance=min(clearance,c);bound=min(bound,min(c(1:end-1),c(2:end))-.5*motion);
end
m.minimum_sampled_road_clearance=min(clearance);m.minimum_interval_road_clearance_bound=min(bound);
% Independent RK4 integration with linearly interpolated controls; five
% substeps per native interval. Report accumulated model drift explicitly.
maxPosition=0;maxHeading=0;
for i=1:p.N
 z=book.state(:,1,i);
 for k=1:numel(book.time)-1
  h=(book.time(k+1)-book.time(k))/5;u0=book.control(:,k,i);u1=book.control(:,k+1,i);
  for sub=1:5
   alpha=(sub-1)/5;ua=u0+alpha*(u1-u0);um=u0+(alpha+.1)*(u1-u0);ub=u0+(alpha+.2)*(u1-u0);
   k1=kinematics(z,ua,g);k2=kinematics(z+h*k1/2,um,g);k3=kinematics(z+h*k2/2,um,g);k4=kinematics(z+h*k3,ub,g);z=z+h*(k1+2*k2+2*k3+k4)/6;
  end
  maxPosition=max(maxPosition,norm(z(1:2)-book.state(1:2,k+1,i)));maxHeading=max(maxHeading,abs(z(3)-book.state(3,k+1,i)));
 end
end
m.maximum_integrated_position_error=maxPosition;m.maximum_integrated_heading_error=maxHeading;
m.integration_check='RK4 with five substeps per native interval and linearly interpolated acceleration and steering rate; discrepancies are measured against the transcribed node states.';
end

function bound(op,lo,x,hi)
% Vectorize inequalities explicitly; avoid matrix-semidefinite semantics.
op.subject_to(x(:)>=lo);op.subject_to(x(:)<=hi);
end
