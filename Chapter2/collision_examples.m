function report=collision_examples()
% Small independent checks for the formulations developed in Chapter 2.
P=[-2,-.5;2,-.5;2,.5;-2,.5];Q=[-.5,-2;.5,-2;.5,2;-.5,2];
assert(sat_gap(P,Q)<0);assert(polygon_distance(P,Q)==0);
for k=1:4,assert(area_residual(P(k,:),Q)>0);assert(area_residual(Q(k,:),P)>0);end
report.crossing_rectangles='All vertices outside the other rectangle; SAT still detects overlap.';
P=[-2,-1;0,-1;0,1;-2,1];Q=[3,-1;5,-1;5,1;3,1];
assert(abs(polygon_distance(P,Q)-3)<1e-12);assert(abs(sat_gap(P,Q)-3)<1e-12);
% Half-space rows [x;-x;y;-y]. Exact unsigned-distance OBCA witness.
A=[1,0;-1,0;0,1;0,-1];bP=[0;2;1;1];bQ=[5;-3;1;1];
lambda=[1;0;0;0];mu=[0;1;0;0];
assert(norm(A'*lambda+A'*mu)<1e-12);assert(norm(A'*lambda)<=1+1e-12);
value=-bP'*lambda-bQ'*mu;assert(abs(value-3)<1e-12);
report.obca_witness_distance_m=value;
% Optimized-plane form is witnessed here by n=(1,0), beta=1.5, margin=3.
n=[1;0];beta=1.5;assert(all(P*n<=beta-1.5));assert(all(Q*n>=beta+1.5));
report.separating_plane_margin_m=3;
% Conservative tangent inequality: affine lower bound never exceeds f.
rng(30);c=10*randn(2000,2);ref=[4,2];r=2.5;
f=sum(c.^2,2)-r^2;ell=sum(ref.^2)-r^2+2*(c-ref)*ref';
assert(all(f-ell>=-1e-10));report.halfspace_checks=2000;
report.passed=true;
end
