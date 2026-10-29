function report=draw_figures(out)
%DRAW_FIGURES Rebuild the chapter's vector figures from explicit geometry.
if nargin==0,out=fullfile(fileparts(mfilename('fullpath')),'output');end
if ~isfolder(out),mkdir(out);end
set(groot,'defaultAxesFontName','Times New Roman','defaultTextFontName','Times New Roman', ...
 'defaultAxesFontSize',16,'defaultTextFontSize',18,'defaultTextInterpreter','tex');
g=struct('LW',2.8,'LF',.96,'LR',.929,'LB',1.942);blue=[.10,.36,.72];teal=[0,.58,.66];gold=[.91,.65,.08];
% 2.1: one coherent physical footprint, indexed dimensions and reference pose.
[f,ax]=canvas(850,560,[-3.4,6.2,-2.9,4.4]);q=[0;0;.27];car(ax,q,g,blue);
R=[cos(q(3)),-sin(q(3));sin(q(3)),cos(q(3))];T=@(v)R*v+q(1:2);
for x=[-g.LR,0,g.LW,g.LW+g.LF]
 line2(ax,T([x;1.10]),T([x;2.05]),[.55,.55,.55],.7,':');
end
for item={{-g.LR,0,'r'},{0,g.LW,'w'},{g.LW,g.LW+g.LF,'f'}}
 z=item{1};a=z{1};b=z{2};dim(ax,T([a;1.80]),T([b;1.80]));
 label(ax,T([(a+b)/2;2.13]),['L_{',z{3},',i}'],18);
end
dim(ax,T([4.45;-g.LB/2]),T([4.45;g.LB/2]));label(ax,T([4.92;0]),'L_{b,i}',18);
for y=[-g.LB/2,g.LB/2],line2(ax,T([3.85;y]),T([4.55;y]),[.55,.55,.55],.7,':');end
P=q(1:2);Q=T([g.LW;0]);plot(ax,[P(1),Q(1)],[P(2),Q(2)],'k.','MarkerSize',14);
label(ax,T([-.1;-.35]),'P',18);label(ax,T([g.LW;-.35]),'Q',18);
label(ax,T([-.05;-1.45]),'(x_i, y_i)',18);
% Equivalent front wheel and steering angle, physically measured from body axis.
phi=.42;w=[cos(q(3)+phi);sin(q(3)+phi)];line2(ax,Q-.42*w,Q+.42*w,[.1,.1,.1],3,'-');
arrow(ax,Q+.1*w,Q+1.4*w,blue,1.3);label(ax,Q+1.52*w,'v_{Q,i}',17);
arc(ax,Q,.78,q(3),q(3)+phi,[.2,.2,.2]);label(ax,Q+1.0*[cos(q(3)+phi/2);sin(q(3)+phi/2)],'\phi_i',17);
line2(ax,P,P+[1.9;0],[.5,.5,.5],.7,'--');arc(ax,P,1.45,0,q(3),[.2,.2,.2]);label(ax,[1.75;.14],'\theta_i',17);
O=[-2.8;-2.25];arrow(ax,O,O+[1.3;0],[.2,.2,.2],1);arrow(ax,O,O+[0;1.2],[.2,.2,.2],1);
label(ax,O+[1.48;0],'X',17);label(ax,O+[0;1.4],'Y',17);label(ax,O+[-.15;-.15],'O',17);
export(f,out,'fig02-01');
% 2.2: rear/front axle normals meet at the instantaneous center.
[f,ax]=canvas(720,590,[-2.2,5.5,-1.9,5.8]);q=[0;0;0];car(ax,q,g,blue);P=[0;0];Q=[g.LW;0];phi=.58;C=[0;g.LW/tan(phi)];
line2(ax,P,C,[.2,.2,.2],1.1,'--');line2(ax,Q,C,[.2,.2,.2],1.1,'--');
plot(ax,[P(1),Q(1),C(1)],[P(2),Q(2),C(2)],'k.','MarkerSize',17);
label(ax,P+[-.22;-.38],'P',20);label(ax,Q+[.16;-.4],'Q',20);label(ax,C+[0;.35],'C',20);
arrow(ax,P+[.1;0],P+[1.8;0],blue,1.5);label(ax,[1.3;-.36],'v_i',19);
w=[cos(phi);sin(phi)];line2(ax,Q-.4*w,Q+.4*w,[.1,.1,.1],3,'-');arrow(ax,Q,Q+1.75*w,blue,1.5);label(ax,Q+2*w,'v_{Q,i}',19);
line2(ax,Q,Q+[1.4;0],[.5,.5,.5],.7,':');arc(ax,Q,1.05,0,phi,[.2,.2,.2]);label(ax,Q+1.25*[cos(phi/2);sin(phi/2)],'\phi_i',19);
dim(ax,[-.55;0],[-.55;C(2)]);label(ax,[-1.05;C(2)/2],'\rho_i',20);
for x=[0,g.LW],line2(ax,[x;-1.05],[x;-1.57],[.5,.5,.5],.7,':');end
dim(ax,[0;-1.42],[g.LW;-1.42]);label(ax,[g.LW/2;-1.75],'L_{w,i}',20);
export(f,out,'fig02-02');
% 2.3: exact physical rectangle, single and multiple circumscribing discs.
f=figure('Color','w','Position',[50,50,1080,400],'Visible','off');
for panel=1:2
 ax=axes(f,'Position',[.035+(panel-1)*.5,.12,.43,.82]);hold(ax,'on');axis(ax,'equal');axis(ax,[-2.0,5,-3.2,3.1]);axis(ax,'off');
 count=1;if panel==2,count=3;end
 L=g.LW+g.LF+g.LR;r=hypot(L/(2*count),g.LB/2);centers=-g.LR+((1:count)-.5)*L/count;
 for j=1:count,circle(ax,[centers(j);0],r,teal,1.1,'-');end
 car(ax,[0;0;0],g,blue);
 for j=1:count,plot(ax,centers(j),0,'k.','MarkerSize',13);end
 if panel==1
  c=[centers;0];dim(ax,c,c+r*[cos(.70);sin(.70)]);label(ax,c+[.9;1.15],'r_i',20);label(ax,c+[0;-.48],'c_i',20);
 else
  for j=1:count,label(ax,[centers(j);-1.38],sprintf('c_i^{(%d)}',j),19);end
  circle(ax,[centers(2);0],r,teal,1.1,'-');dim(ax,[centers(2);0],[centers(2);r]);label(ax,[centers(2)+.4;r*.70],'r_i^{(3)}',19);
 end
 label(ax,[1.4;-2.8],['(',char(96+panel),') ',char(string(count)),' disc',repmat('s',1,count>1)],21);
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
 plot(ax,p(1),p(2),'k.','MarkerSize',16);label(ax,p'+[0;-.32],'p',19);label(ax,[.45;-.85],name,18);
end
export(f,out,'fig02-04');
% 2.5: actual radius inflation, followed by accepted/rejected full-strip tests.
obs={[-6,1;-4.2,.6;-2.3,1.7;-2.7,4.4;-5.3,5.6;-6.8,3.7], ...
 [2.3,1.9;5.8,.9;6.6,4.3;4.4,6.2;2.2,4.5], ...
 [.8,-4.7;6.6,-4.7;6.7,-2.1;3.8,-1;2,-2]};
r=.65;seed=[-.4,.15];bounds=[-7.5,7.5,-5.5,7];[b,h]=grow_corridor(seed,obs,r,bounds,.24);
f=figure('Color','w','Position',[50,50,1170,435],'Visible','off');
for panel=1:3
 ax=axes(f,'Position',[.015+(panel-1)*.333,.13,.318,.84]);hold(ax,'on');axis(ax,'equal');axis(ax,bounds);axis(ax,'off');scene(ax,obs,r);
 if panel>=2
  if panel==2
   acc=h(h(:,2)==1,3:6);last=size(acc,1);for j=unique([3,round(last/3),round(2*last/3)]),box(ax,acc(j,:),[.62,.73,.82],.8,'-');end
  end
  box(ax,b,blue,1.9,'-');
 end
 if panel==2
  rejected=h(h(:,2)==0,3:6);for j=1:size(rejected,1),box(ax,rejected(j,:),[.78,.35,.24],.85,'--');end
 elseif panel==3
  line2(ax,[b(1);seed(2)],[b(2);seed(2)],blue,.6,':');line2(ax,[seed(1);b(3)],[seed(1);b(4)],blue,.6,':');
  label(ax,[mean(b(1:2));b(4)+.45],'Upper bound',20);label(ax,[mean(b(1:2));b(3)-.5],'Lower bound',20);
 end
 plot(ax,seed(1),seed(2),'ko','MarkerFaceColor','w','MarkerSize',5,'LineWidth',1);
 names={'(a) Disc-radius inflation','(b) Cyclic box expansion','(c) Accepted corridor'};label(ax,[0;-5.15],names{panel},23);
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
f=figure('Color','w','Position',[50,50,1020,410],'Visible','off');P=[-3,-1;-1,-1;-.4,.8;-2.5,1.5];Q=[1,.3;2.8,-.2;3.1,1.7;1.4,2.3];
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
  beta=.3;line2(ax,[beta;-1.6],[beta;2.6],[.25,.25,.25],1.3,'-');
  line2(ax,[-.4;-1.6],[-.4;2.6],blue,.8,'--');line2(ax,[1;-1.6],[1;2.6],gold,.8,'--');
  arrow(ax,[beta;2.45],[1.65;2.45],[.25,.25,.25],1);label(ax,[1.85;2.45],'n',19);
  dim(ax,[-.4;-1.3],[1;-1.3]);label(ax,[.3;-1.65],'d_{safe}',17);label(ax,[0;-2.2],'(b) Separating line and margin',20);
 end
end
export(f,out,'fig02-08');
report=collision_examples();report.stc=struct('radius',r,'seed',seed,'box',b,'history',h,'corridors',records);
report.figures={'fig02-01','fig02-02','fig02-03','fig02-04','fig02-05','fig02-06','fig02-08'};
file=fopen(fullfile(out,'validation.json'),'w');fprintf(file,'%s',jsonencode(report));fclose(file);
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
