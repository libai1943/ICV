function book=run_intro_intersection()
% Dense 24-vehicle through traffic on the same 7 m road.
% Geometry, route seeds and traffic task differ from the published benchmark.

root=fileparts(mfilename('fullpath'));addpath(fullfile(root,'common'),fullfile(root,'ch05_intersection'));
solverPath=getenv('CASADI_MATLAB_PATH');assert(isfolder(solverPath),'Set CASADI_MATLAB_PATH.');addpath(solverPath);assert(isfile(getenv('IPOPT_HSL_LIBRARY')),'Set IPOPT_HSL_LIBRARY to an HSL library containing MA57.');
scene=CreateScenario();p=scene.parameters;p.nodes=121;p.horizon=16;p.dt=p.horizon/(p.nodes-1);p.vmin=0;p.vmax=5;p.phimax=.65;p.omegamax=.6;p.box_step=.10;p.box_limit=3;p.max_iterations=6;p.road_width=7;scene.parameters=p;
scene.workspace=[-1000,1000,-1000,1000];scene.obstacles=[-1000,-3.5,-1000,-3.5;3.5,1000,-1000,-3.5;-1000,-3.5,3.5,1000;3.5,1000,3.5,1000];
vehicleCount=24;scene.vehicles=repmat(scene.vehicles(1),1,vehicleCount);turns=repmat([1,0,-1,0,-1,1,0,1,0,-1,1,-1],1,2);headings=[0,pi/2,pi,-pi/2];reference=zeros(9,p.nodes,vehicleCount);outcount=zeros(1,4);tt=linspace(0,p.horizon,p.nodes);
for i=1:vehicleCount
 fprintf('Route %d\n',i);
 a=headings(mod(i-1,4)+1);rot=[cos(a),-sin(a);sin(a),cos(a)];g=struct('LW',2.8,'LF',.96,'LR',.929,'LB',1.942);g.offset=(g.LW+g.LF-g.LR)/2;g.radius=hypot(g.LW+g.LF+g.LR,g.LB)/2;
 startx=-14;turn=turns(i);
 if turn==1,r=5.25;corner=3.5;arclen=r*pi/2; elseif turn==-1,r=4.25;corner=6;arclen=r*pi/2;else,r=0;corner=3.5;arclen=7;end
 approach=-corner-startx;outid=mod(mod(i-1,4)+turn,4)+1;outcount(outid)=outcount(outid)+1;total=approach+arclen+14-corner;release=0;duration=p.horizon;u=min(max((tt-release)/duration,0),1);distance=2*tt+(total-2*p.horizon)*(u-sin(2*pi*u)/(2*pi));xy=zeros(2,p.nodes);theta=zeros(1,p.nodes);
 fprintf('Route coordinates %d\n',i);
 for k=1:p.nodes
  d=distance(k);
  if d<approach,xy(:,k)=[startx+d;-1.75];
  elseif d<approach+arclen
   z=d-approach;
   if turn==1,b=z/r;xy(:,k)=[-corner+r*sin(b);corner-r*cos(b)];theta(k)=b;
   elseif turn==-1,b=z/r;xy(:,k)=[-corner+r*sin(b);-corner+r*cos(b)];theta(k)=-b;
   else,xy(:,k)=[-corner+z;-1.75];end
  else
   z=d-approach-arclen;
   if turn==1,xy(:,k)=[1.75;corner+z];theta(k)=pi/2;
   elseif turn==-1,xy(:,k)=[-1.75;-corner-z];theta(k)=-pi/2;
   else,xy(:,k)=[corner+z;-1.75];end
  end
 end
 fprintf('Route derivatives %d\n',i);
 xy=[rot(1,1)*xy(1,:)+rot(1,2)*xy(2,:);rot(2,1)*xy(1,:)+rot(2,2)*xy(2,:)];

theta=theta+a;speed=2+(total/p.horizon-2)*(1-cos(2*pi*u));

g.heading=a+turn*pi/2;g.initial=zeros(9,1);g.initial(1:2)=xy(:,1);g.initial(3)=2;g.initial(7)=a;g.initial(8:9)=xy(:,1)+g.offset*[cos(a);sin(a)];
 g.exit=xy(:,end)-15*[cos(g.heading);sin(g.heading)];g.desired=xy(:,end);g.exit_speed=2;scene.vehicles(i)=g;


 reference(1:2,:,i)=xy;reference(3,:,i)=speed;reference(4,:,i)=[diff(speed),0]/p.dt;reference(7,:,i)=theta;

reference(5,:,i)=atan(g.LW*[diff(theta),0]/p.dt./max(speed,.1));

reference(6,:,i)=[diff(reference(5,:,i)),0]/p.dt;reference(8:9,:,i)=xy+g.offset*[cos(theta);sin(theta)];
end
out=fullfile(root,'intro_results/intersection_mixed');if ~isfolder(out),mkdir(out);end
save(fullfile(out,'scenario.mat'),'scene','reference');timer=tic;
% Solve each kinematic maneuver using direct transcription and the upstream
% STC generator, then coordinate
% its departure time by collision-checked priority scheduling. This illustration
% is not presented as a jointly time-optimal result or as a new benchmark.
paths=cell(1,vehicleCount);
for i=1:vehicleCount
 single=scene;single.vehicles=scene.vehicles(i);fprintf('Single maneuver %d\n',i);
 rr=SolveIntroManeuver(single,reference(:,:,i));paths{i}=rr.fleet;save(fullfile(out,sprintf('single_%02d.mat',i)),'rr','single');
end
book=ScheduleThroughTraffic(scene,paths);book=AdvanceIncomingQueues(book);book.planning_seconds=toc(timer);book.method='Hard bicycle-model maneuvers, advancing queues, and continuing outbound traffic with rectangle-checked time coordination';
save(fullfile(out,'book_result.mat'),'book','paths','-v7.3');disp(rmfield(book.validation,{'sampled_gap','interval_bound'}));
end
