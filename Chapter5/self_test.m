function self_test()
P=[1,-1;3,-1;4,0;3,1;1,1];assert(abs(polygon_box_distance([-1,0,-.5,.5],P)-1)<1e-8);assert(polygon_box_distance([2,2.5,-.3,.3],P)==0);assert(polygon_box_distance([0,1,-2,-1],P)<1e-12);
box=[-2,2,-2,2];d=hypot(max([box(1),-box(2),0]),max([box(3),-box(4),0]));assert(d==0&&hypot(2,2)>1);
q=[0;0;0;2;0;.3];u=[.2;0];h=.4;next=euler_step(q,u,h,2.8);assert(abs(next(1)-(2*h))<1e-12);
scene.geometry=struct('LW',2.8,'LF',.96,'LR',.929,'LB',1.942);[radius,offsets]=disc_geometry(scene);[x,y]=meshgrid(linspace(-.929,3.76,101),linspace(-.971,.971,51));distance=inf(size(x));for d=1:4,distance=min(distance,hypot(x-offsets(d),y));end;assert(max(distance,[],'all')<=radius+1e-12);
points=[-1,0;2,0;5,0];assert(max(abs(point_polygon_distance(points,P)-[2;0;1]))<1e-10);
fprintf('PASS: polygon/box distance, relative-box interior, forward Euler state update.\n');
end
