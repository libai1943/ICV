function W=rounded_offset(P,r)
% Plotting boundary of P plus a radius-r disc, for a convex CCW polygon.
% Circular corner arcs are sampled for vector drawing only; clearance tests
% use polygon_distance on original polygons and the exact radius.
if sum(P(:,1).*P([2:end,1],2)-P([2:end,1],1).*P(:,2))<0,P=flipud(P);end
E=P([2:end,1],:)-P;n=[E(:,2),-E(:,1)];n=n./sqrt(sum(n.^2,2));W=[];
for k=1:size(P,1)
 j=mod(k-2,size(P,1))+1;a=atan2(n(j,2),n(j,1));b=atan2(n(k,2),n(k,1));delta=mod(b-a,2*pi);
 assert(delta<pi+1e-8,'A strictly convex polygon is required.');
 angle=linspace(a,a+delta,max(3,ceil(delta/.025)));
 W=[W;P(k,:)+r*[cos(angle(:)),sin(angle(:))]]; %#ok<AGROW>
end
end
