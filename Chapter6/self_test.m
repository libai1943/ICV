function self_test()
task=make_case(1);g=task.geometry;length=g.LW+g.LF+g.LR;radius=hypot(length/4,g.LB/2);centers=[-g.LR+length/4,-g.LR+3*length/4];
for x=linspace(-g.LR,g.LW+g.LF,101),for y=[-g.LB/2,g.LB/2],assert(min(hypot(x-centers,y))<=radius+1e-12);end,end
q=[0;0;0;2;0;.3];h=.4;next=euler_step(q,[.2;0],h,g.LW);assert(abs(next(1)-(2*h))<1e-12);
for n=1:8,t=make_case(n);assert(isequal(size(t.start),[3,10])&&isequal(size(t.goal),[3,10])&&isequal(size(t.obstacles),[5,3]));end
q=[1;2;.3;1;.2;.4];u=[.2;.1];h=.4;scale=1.2;a=euler_step(q,u,h,g.LW);qt=q;qt(4)=qt(4)/scale;qt(6)=qt(6)/scale^2;ut=[u(1)/scale^3;u(2)/scale];b=euler_step(qt,ut,h*scale,g.LW);a(4)=a(4)/scale;a(6)=a(6)/scale^2;assert(max(abs(a-b))<1e-12);
fprintf('PASS: two-disc body cover, forward Euler state update, eight case schemas, uniform time-dilation invariance.\n');
end
