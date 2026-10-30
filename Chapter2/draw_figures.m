function report=draw_figures(out)
%DRAW_FIGURES Rebuild the chapter's vector figures from explicit geometry.
if nargin==0,out=fullfile(fileparts(mfilename('fullpath')),'output');end
if ~isfolder(out),mkdir(out);end
set(groot,'defaultAxesFontName','Times New Roman','defaultTextFontName','Times New Roman', ...
 'defaultAxesFontSize',16,'defaultTextFontSize',18,'defaultTextInterpreter','tex');
g=struct('LW',2.8,'LF',.96,'LR',.929,'LB',1.942);blue=[.10,.36,.72];teal=[0,.58,.66];gold=[.91,.65,.08];
% 2.1: physical wheels, equivalent axle-center wheels, and complete angle rays.
[f,ax]=canvas(1000,760,[-3.5,6.0,-3.4,5.5]);q=[0;0;.48];phi=.38;
R=[cos(q(3)),-sin(q(3));sin(q(3)),cos(q(3))];T=@(v)R*v;
car(ax,q,g,blue);P=T([0;0]);Q=T([g.LW;0]);
% Four physical wheels, with a narrow highlight to distinguish their orientation.
for x=[0,g.LW]
 steer=0;if x>0,steer=phi;end
 for y=[-g.LB/2,g.LB/2]
  wheel(ax,T([x;y]),q(3)+steer,false);
  arrow(ax,T([x;.76*y]),T([x;.26*sign(y)]),[.42,.42,.42],.8);
 end
 wheel(ax,T([x;0]),q(3)+steer,true);
end
line2(ax,T([-1.45;0]),T([5.1;0]),[.22,.22,.22],.95,'-.');
line2(ax,T([0;-1.25]),T([0;1.2]),[.45,.45,.45],.7,':');
line2(ax,T([g.LW;-1.25]),T([g.LW;1.2]),[.45,.45,.45],.7,':');
for x=[-g.LR,0,g.LW,g.LW+g.LF]
 line2(ax,T([x;1.14]),T([x;2.13]),[.55,.55,.55],.7,':');
end
for item={{-g.LR,0,'r'},{0,g.LW,'w'},{g.LW,g.LW+g.LF,'f'}}
 z=item{1};dim(ax,T([z{1};1.86]),T([z{2};1.86]));
 label(ax,T([(z{1}+z{2})/2;2.18]),['L_{',z{3},',i}'],22);
end
dim(ax,T([4.66;-g.LB/2]),T([4.66;g.LB/2]));label(ax,T([5.08;0]),'L_{b,i}',22);
for y=[-g.LB/2,g.LB/2],line2(ax,T([3.9;y]),T([4.82;y]),[.55,.55,.55],.7,':');end
O=[-2.7;-2.7];arrow(ax,O,[5.4;O(2)],[.25,.25,.25],.9);arrow(ax,O,[O(1);3.5],[.25,.25,.25],.9);
label(ax,[5.65;O(2)],'X',21);label(ax,[O(1);3.8],'Y',21);label(ax,O+[-.18;-.2],'O',20);
line2(ax,[0;O(2)],P,[.5,.5,.5],.8,':');line2(ax,[O(1);0],P,[.5,.5,.5],.8,':');
label(ax,[0;O(2)-.3],'\it x_i',21);label(ax,[O(1)-.32;0],'\it y_i',21);
% theta: horizontal ray at P versus vehicle longitudinal ray at P.
line2(ax,P,P+[2.1;0],[.3,.3,.3],.8,'--');arc(ax,P,1.65,0,q(3),[.15,.15,.15]);
label(ax,1.92*[cos(q(3)/2);sin(q(3)/2)],'\theta_i',22);
% phi: longitudinal ray at Q versus steered equivalent-wheel ray at Q.
w=[cos(q(3)+phi);sin(q(3)+phi)];line2(ax,Q-.55*w,Q+1.62*w,blue,1,'--');
arc(ax,Q,1.25,q(3),q(3)+phi,[.15,.15,.15]);
label(ax,Q+1.52*[cos(q(3)+phi/2);sin(q(3)+phi/2)],'\phi_i',22);
plot(ax,[P(1),Q(1)],[P(2),Q(2)],'ko','MarkerFaceColor','k','MarkerSize',4);
label(ax,T([-.33;-.38]),'P',23);label(ax,T([g.LW+.1;-.38]),'Q',23);
label(ax,[-1.2;-1.55],'Equivalent rear wheel',19);line2(ax,[-.8;-1.3],P+[-.12;-.12],[.3,.3,.3],.7,'-');
label(ax,[2.65;-1.65],'Equivalent front wheel',19);line2(ax,[2.65;-1.4],Q+[.07;-.1],[.3,.3,.3],.7,'-');

export(f,out,'fig02-01');
% 2.2: rotate every point, ray, and dimension with the same rigid transform.
[f,ax]=canvas(820,690,[-3.4,5.6,-2.7,5.5]);ang=.34;R=[cos(ang),-sin(ang);sin(ang),cos(ang)];T=@(v)R*v;
phi=.58;rho=g.LW/tan(phi);P=T([0;0]);Q=T([g.LW;0]);C=T([0;rho]);car(ax,[0;0;ang],g,blue);
line2(ax,T([-1.2;0]),T([4.4;0]),[.4,.4,.4],.8,'-.');line2(ax,P,C,[.2,.2,.2],1.1,'--');line2(ax,Q,C,[.2,.2,.2],1.1,'--');
plot(ax,[P(1),Q(1),C(1)],[P(2),Q(2),C(2)],'k.','MarkerSize',17);
label(ax,T([-.25;-.4]),'P',22);label(ax,T([g.LW+.1;-.4]),'Q',22);label(ax,C+[0;.35],'C',22);
arrow(ax,P,T([1.8;0]),blue,1.4);label(ax,T([1.1;-.36]),'v_i',21);
w=R*[cos(phi);sin(phi)];wheel(ax,Q,ang+phi,true);arrow(ax,Q,Q+1.9*w,blue,1.4);label(ax,Q+2.2*w,'v_{Q,i}',21);
arc(ax,Q,1.15,ang,ang+phi,[.2,.2,.2]);label(ax,Q+1.4*[cos(ang+phi/2);sin(ang+phi/2)],'\phi_i',21);
dim(ax,T([-.57;0]),T([-.57;rho]));label(ax,T([-1.03;rho/2]),'\rho_i',22);
for x=[0,g.LW],line2(ax,T([x;-1.06]),T([x;-1.6]),[.5,.5,.5],.7,':');end
dim(ax,T([0;-1.43]),T([g.LW;-1.43]));label(ax,T([g.LW/2;-1.8]),'L_{w,i}',22);
% Right-angle markers establish the velocity/normal construction explicitly.
mark=T([0;.28]);line2(ax,mark,mark+R*[.28;0],[.3,.3,.3],.8,'-');line2(ax,mark+R*[.28;0],T([.28;0]),[.3,.3,.3],.8,'-');
n=(C-Q)/norm(C-Q);mark=Q+.28*n;line2(ax,mark,mark+.28*w,[.3,.3,.3],.8,'-');line2(ax,mark+.28*w,Q+.28*w,[.3,.3,.3],.8,'-');
export(f,out,'fig02-02');
% 2.3: rotated body covers with outlines on the top layer.
f=figure('Color','w','Position',[50,50,1080,470],'Visible','off');
for panel=1:2
 ax=axes(f,'Position',[.025+(panel-1)*.5,.04,.45,.93]);hold(ax,'on');axis(ax,'equal');axis(ax,[-2.2,5.2,-3.3,3.9]);axis(ax,'off');
 count=1;if panel==2,count=3;end
 ang=.36;R=[cos(ang),-sin(ang);sin(ang),cos(ang)];T=@(v)R*v;
 L=g.LW+g.LF+g.LR;r=hypot(L/(2*count),g.LB/2);centers=-g.LR+((1:count)-.5)*L/count;
 car(ax,[0;0;ang],g,blue);
 line2(ax,T([-1.2;0]),T([4;0]),[.45,.45,.45],.7,'-.');
 for j=1:count,c=T([centers(j);0]);circle(ax,c,r,teal,1.25,'-');plot(ax,c(1),c(2),'k.','MarkerSize',13);end
 if panel==1
  c=T([centers;0]);w=[cos(1.15);sin(1.15)];dim(ax,c,c+r*w);label(ax,c+r*.65*w+[.3;.1],'r_i',22);label(ax,c+[0;-.43],'c_i',22);
 else
  for j=1:count,label(ax,T([centers(j);-1.65]),sprintf('c_i^{(%d)}',j),21);end
  c=T([centers(2);0]);dim(ax,c,c+R*[0;r]);label(ax,c+R*[.45;r*.75],'r_i^{(3)}',21);
 end
 names={'(a) Single-disc cover','(b) Three-disc cover'};label(ax,[1.5;-2.9],names{panel},18);
end
export(f,out,'fig02-03');
% 2.4: point-area criterion and its limitation, shown without raster labels.
f=figure('Color','w','Position',[50,50,1000,395],'Visible','off');
P=[0,0;2.4,0;3.1,1.8;1.4,3;-.6,1.7];
for panel=1:2
 ax=axes(f,'Position',[.035+(panel-1)*.5,.12,.43,.82]);hold(ax,'on');axis(ax,'equal');axis(ax,[-2.8,3.8,-1.1,3.8]);axis(ax,'off');
 if panel==1
  p=[1.1,1.3];name='(a) Point inside';
 else,p=[-2.1,.4];name='(b) Point outside';end
 patch(ax,P(:,1),P(:,2),[.91,.94,.96],'EdgeColor',blue,'LineWidth',1.3);
 for j=1:size(P,1),line2(ax,p',P(j,:)',[.45,.45,.45],.8,'--');label(ax,P(j,:)'+.25*(P(j,:)'-[1.1;1.3]),sprintf('Q_%d',j),18);end
 plot(ax,p(1),p(2),'k.','MarkerSize',16);label(ax,p'+[0;-.32],'p',19);label(ax,[.45;-.85],name,17);
end
export(f,out,'fig02-04');
% 2.5: actual radius inflation, followed by accepted/rejected full-strip tests.
obs={[-6,1;-4.2,.6;-2.3,1.7;-2.7,4.4;-5.3,5.6;-6.8,3.7], ...
 [2.3,1.9;5.8,.9;6.6,4.3;4.4,6.2;2.2,4.5], ...
 [.8,-4.7;6.6,-4.7;6.7,-2.1;3.8,-1;2,-2]};
r=.65;seed=[-.4,.15];bounds=[-7.5,7.5,-5.5,7];[b,h]=grow_corridor(seed,obs,r,bounds,.85);
f=figure('Color','w','Position',[50,50,1170,435],'Visible','off');
for panel=1:3
 ax=axes(f,'Position',[.015+(panel-1)*.333,.13,.318,.84]);hold(ax,'on');axis(ax,'equal');axis(ax,[-7.5,7.5,-7,8]);axis(ax,'off');scene(ax,obs,r);if panel==2,axis(ax,[-3.4,3.4,-2.8,3.0]);end
 if panel>=2
  box(ax,b,blue,1.9,'-');
 end
 if panel==2
  previous=[seed(1),seed(1),seed(2),seed(2)];
  for k=1:size(h,1)
   side=h(k,1);trial=h(k,3:6);strip=trial;
   if side==1,strip(2)=previous(1);elseif side==2,strip(1)=previous(2);elseif side==3,strip(4)=previous(3);else,strip(3)=previous(4);end
   col=[.40,.56,.67];sty='-';if ~h(k,2),col=[.70,.29,.22];sty='--';end
   box(ax,strip,col,.75,sty);
   mid=[mean(strip(1:2));mean(strip(3:4))];if k==1,mid=mid+[-.16;-.10];end;label(ax,mid,num2str(k),16);
   if h(k,2),previous=trial;end
  end
 elseif panel==3
  line2(ax,[b(1);seed(2)],[b(2);seed(2)],blue,.6,':');line2(ax,[seed(1);b(3)],[seed(1);b(4)],blue,.6,':');

 end
 plot(ax,seed(1),seed(2),'ko','MarkerFaceColor','w','MarkerSize',5,'LineWidth',1);
 if panel~=2,label(ax,seed'+[-.25;-.7],'Seed',21);end
 names={'(a) Disc-radius inflation','(b) Cyclic box expansion','(c) Accepted corridor'};annotation(f,'textbox',[.015+(panel-1)*.333,.015,.318,.07],'String',names{panel},'EdgeColor','none','HorizontalAlignment','center','FontSize',23,'FontName','Times New Roman');
end
export(f,out,'fig02-05');
% 2.6: white corridor interiors versus omitted free space in the same map.
[f,ax]=canvas(790,565,bounds);set(ax,'Color',[.95,.95,.95]);scene(ax,obs,r);
seeds=[-5,-3.3;-2.6,-1.9;-.4,.15;.1,2.5;.2,5.0];records=[];
for k=1:size(seeds,1)
 [bb,~]=grow_corridor(seeds(k,:),obs,r,bounds,.12);records(k,:)=bb; %#ok<AGROW>
 V=[bb(1),bb(3);bb(2),bb(3);bb(2),bb(4);bb(1),bb(4)];patch(ax,V(:,1),V(:,2),[.92,.96,1],'FaceAlpha',.4,'EdgeColor',blue,'LineWidth',1.1);
end
plot(ax,seeds(:,1),seeds(:,2),'k--o','MarkerFaceColor','w','LineWidth',1);
export(f,out,'fig02-06');
% 2.8: candidate axes and a continuous separating plane for convex sets.
f=figure('Color','w','Position',[50,50,1020,410],'Visible','off');P=[-3,-1;-1,-1;-1,1;-3,1];Q=[1,0;3,0;3,2;1,2];
for panel=1:2
 ax=axes(f,'Position',[.035+(panel-1)*.5,.10,.43,.87]);hold(ax,'on');axis(ax,'equal');axis(ax,[-3.8,3.8,-2.4,3]);axis(ax,'off');
 patch(ax,P(:,1),P(:,2),[.86,.91,.97],'EdgeColor',blue,'LineWidth',1.3);patch(ax,Q(:,1),Q(:,2),[.98,.92,.78],'EdgeColor',gold,'LineWidth',1.3);
 label(ax,[-1.9;0],'P',20);label(ax,[2;1],'Q',20);
 if panel==1
  arrow(ax,[-3.4;-1.55],[3.3;-1.55],[.25,.25,.25],.9);
  for x=[min(P(:,1)),max(P(:,1)),min(Q(:,1)),max(Q(:,1))],line2(ax,[x;-.5],[x;-1.55],[.6,.6,.6],.65,':');end
  line2(ax,[min(P(:,1));-1.55],[max(P(:,1));-1.55],blue,4,'-');line2(ax,[min(Q(:,1));-1.55],[max(Q(:,1));-1.55],gold,4,'-');
  label(ax,[.3;-1.85],'gap',17);label(ax,[0;-2.2],'(a) Projection intervals',20);
 else
  beta=0;line2(ax,[beta;-1.6],[beta;2.6],[.25,.25,.25],1.3,'-');
  line2(ax,[-1;-1.6],[-1;2.6],blue,.8,'--');line2(ax,[1;-1.6],[1;2.6],gold,.8,'--');
  arrow(ax,[beta;2.45],[1.65;2.45],[.25,.25,.25],1);label(ax,[1.85;2.45],'n',19);
  dim(ax,[-1;-1.3],[1;-1.3]);label(ax,[0;-1.65],'Projected gap',17);label(ax,[0;-2.2],'(b) Separating line and margin',20);
 end
end
export(f,out,'fig02-08');
% 2.7: preserve the half-space example and add normal/radius construction lines.
f=figure('Color','w','Position',[50,50,1040,510],'Visible','off');
for panel=1:2
 ax=axes(f,'Position',[.025+(panel-1)*.5,.04,.45,.93]);hold(ax,'on');axis(ax,'equal');axis(ax,[-4.1,6,-4.1,5.6]);axis(ax,'off');
 c=[4;2];n=c/norm(c);tangent=[-n(2);n(1)];foot=(26.25/2)/dot(c,c)*c;
 if panel==1
  patch(ax,[-4,6,6,-4],[-3.5,-3.5,5.1,5.1],[.94,.96,.98],'EdgeColor','none');
  aa=linspace(0,2*pi,241);patch(ax,2.5*cos(aa),2.5*sin(aa),'w','EdgeColor','none');
 else
  V=[foot-7*tangent,foot+7*tangent,foot+7*tangent+9*n,foot-7*tangent+9*n];patch(ax,V(1,:),V(2,:),[.94,.96,.98],'EdgeColor','none');
  line2(ax,foot-7*tangent,foot+7*tangent,blue,1.3,'-');
 end
 aa=linspace(0,2*pi,241);patch(ax,2*cos(aa),2*sin(aa),[.66,.66,.66],'EdgeColor',[.25,.25,.25]);circle(ax,[0;0],2.5,[.3,.3,.3],1,'--');circle(ax,c,.5,blue,1.1,'-');
 plot(ax,[0,c(1)],[0,c(2)],'k.','MarkerSize',14);label(ax,[-.25;-.35],'o',21);label(ax,c+[.2;.85],'\it c',22);line2(ax,c+[.08;1.10],c+[.32;1.10],[.2,.2,.2],.8,'-');
 line2(ax,[0;0],c,[.3,.3,.3],.85,'-.');arrow(ax,[0;0],2*n,[.3,.3,.3],.8);
 if panel==2
  a=foot+.3*tangent;line2(ax,a,a+.3*n,[.3,.3,.3],.9,'-');line2(ax,a+.3*n,foot+.3*n,[.3,.3,.3],.9,'-');
  arrow(ax,foot+1.1*tangent,foot+1.1*tangent+1.2*n,blue,1);label(ax,foot+1.1*tangent+1.5*n,'n',21);
  label(ax,[2;-1.65],'8x_c + 4y_c = 26.25',19);
 end
 names={'(a) Nominal feasible region','(b) Conservative half-space'};label(ax,[.95;-3.8],names{panel},22);
end
export(f,out,'fig02-07');
% 2.9: active supporting faces and their opposing dual normals.
[f,ax]=canvas(1000,450,[-3.2,6.2,-2.3,2.4]);
patch(ax,[-2,0,0,-2],[-1,-1,1,1],[.86,.91,.97],'EdgeColor',blue,'LineWidth',1.2);
patch(ax,[3,5,5,3],[-1,-1,1,1],[.98,.92,.78],'EdgeColor',gold,'LineWidth',1.2);
line2(ax,[0;-1],[0;1],blue,2.5,'-');line2(ax,[3;-1],[3;1],gold,2.5,'-');
line2(ax,[1.5;-1.7],[1.5;1.7],[.35,.35,.35],1,'--');
arrow(ax,[0;.45],[1;.45],blue,1.3);arrow(ax,[3;.45],[2;.45],gold,1.3);
label(ax,[.6;.95],'A_P^T\lambda = (1,0)^T',19);label(ax,[3.1;1.5],'A_Q^T\mu = (-1,0)^T',19);
label(ax,[-1;0],'P',23);label(ax,[4;0],'Q',23);
plot(ax,[0,3],[-.3,-.3],'ko','MarkerFaceColor','k','MarkerSize',4);dim(ax,[0;-.3],[3;-.3]);label(ax,[1.5;-.7],'Distance = 3 m',20);
label(ax,[1.5;-2],'Dual value = -b_P^T\lambda - b_Q^T\mu = 3 m',21);
export(f,out,'fig02-09');
report=collision_examples();report.stc=struct('radius',r,'seed',seed,'box',b,'history',h,'corridors',records);
report.figures={'fig02-01','fig02-02','fig02-03','fig02-04','fig02-05','fig02-06','fig02-07','fig02-08','fig02-09'};
file=fopen(fullfile(out,'geometry_validation.json'),'w');fprintf(file,'%s',jsonencode(report));fclose(file);
end
function [f,ax]=canvas(w,h,limits)
f=figure('Color','w','Position',[50,50,w,h],'Visible','off');ax=axes(f,'Position',[.02,.02,.96,.96]);hold(ax,'on');axis(ax,'equal');axis(ax,limits);axis(ax,'off');
end
function car(ax,q,g,color)
V=rectangle_vertices(q,g);patch(ax,V(:,1),V(:,2),.80+.20*color,'EdgeColor',color,'LineWidth',1.3);
L=g.LW+g.LF+g.LR;R=[cos(q(3)),-sin(q(3));sin(q(3)),cos(q(3))];V=(R*[-g.LR+.29*L,-g.LR+.29*L,-g.LR+.64*L;-g.LB/2,g.LB/2,0]+q(1:2))';
patch(ax,V(:,1),V(:,2),.6+.4*color,'EdgeColor',color,'LineWidth',1);
end
function label(ax,p,s,sz)
% Explicit upright geometric parameters, italic running index and state symbols.
s=regexprep(s,'L_\{([rwfb]),i\}','{\\rm L}_{\\rm $1,\\it i}');
s=regexprep(s,'^r_i','{\\rm r}_{\\it i}');
s=regexprep(s,'^c_i','{\\bf c}_{\\it i}');
s=strrep(s,'(x_i, y_i)','(\it x_i, y_i\rm)');
if startsWith(s,'v'),s=['\it ',s];end
s=strrep(s,'\phi_i','\phi_{\it i}');s=strrep(s,'\theta_i','\theta_{\it i}');s=strrep(s,'\rho_i','\rho_{\it i}');
text(ax,p(1),p(2),s,'FontName','Times New Roman','FontSize',sz,'HorizontalAlignment','center','VerticalAlignment','middle','Color','k','BackgroundColor','none','Interpreter','tex');
end
function line2(ax,a,b,c,w,st)
plot(ax,[a(1),b(1)],[a(2),b(2)],st,'Color',c,'LineWidth',w);
end
function arrow(ax,a,b,c,w)
line2(ax,a,b,c,w,'-');u=(b-a)/norm(b-a);n=[-u(2);u(1)];sz=.11;P=[b,b-sz*u+.45*sz*n,b-sz*u-.45*sz*n];patch(ax,P(1,:),P(2,:),c,'EdgeColor',c);
end
function dim(ax,a,b)
arrow(ax,a,b,[.25,.25,.25],.75);arrow(ax,b,a,[.25,.25,.25],.75);
end
function circle(ax,c,r,color,w,st)
t=linspace(0,2*pi,241);plot(ax,c(1)+r*cos(t),c(2)+r*sin(t),st,'Color',color,'LineWidth',w);
end
function arc(ax,c,r,a,b,col)
t=linspace(a,b,40);plot(ax,c(1)+r*cos(t),c(2)+r*sin(t),'-','Color',col,'LineWidth',.9);
end
function box(ax,b,c,w,st)
plot(ax,b([1,2,2,1,1]),b([3,3,4,4,3]),st,'Color',c,'LineWidth',w);
end
function scene(ax,obs,r)
for j=1:numel(obs),P=rounded_offset(obs{j},r);patch(ax,P(:,1),P(:,2),[.9,.9,.9],'EdgeColor',[.65,.65,.65],'LineWidth',.65);end
for j=1:numel(obs),P=obs{j};patch(ax,P(:,1),P(:,2),[.63,.63,.63],'EdgeColor',[.3,.3,.3],'LineWidth',.75);end
end
function export(f,out,name)
exportgraphics(f,fullfile(out,[name,'.pdf']),'ContentType','vector');exportgraphics(f,fullfile(out,[name,'.png']),'Resolution',220);close(f);
end

function wheel(ax,c,angle,equivalent)
R=[cos(angle),-sin(angle);sin(angle),cos(angle)];V=R*[-.37,.37,.37,-.37;-.115,-.115,.115,.115]+c;
if equivalent,col=[.98,.98,.98];sty='--';else,col=[.23,.25,.28];sty='-';end
patch(ax,V(1,:),V(2,:),col,'EdgeColor',[.12,.12,.12],'LineWidth',1.0,'LineStyle',sty);
if ~equivalent,V=R*[-.30,.30,.30,-.30;-.065,-.065,-.015,-.015]+c;patch(ax,V(1,:),V(2,:),[.68,.70,.73],'EdgeColor','none');end
end
