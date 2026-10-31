function self_test()
P=[1,-1;3,-1;4,0;3,1;1,1];assert(abs(polygon_box_distance([-1,0,-.5,.5],P)-1)<1e-8);assert(polygon_box_distance([2,2.5,-.3,.3],P)==0);assert(polygon_box_distance([0,1,-2,-1],P)<1e-12);
box=[-2,2,-2,2];d=hypot(max([box(1),-box(2),0]),max([box(3),-box(4),0]));assert(d==0&&hypot(2,2)>1);
q=[0;0;0;2;0;.3];u=[.2;0];h=.4;next=euler_step(q,u,h,2.8);assert(abs(next(1)-(2*h))<1e-12);
fprintf('PASS: polygon/box distance, relative-box interior, forward Euler state update.\n');
end
