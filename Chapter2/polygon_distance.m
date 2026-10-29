function d=polygon_distance(P,Q)
% Exact Euclidean distance between two convex polygons (roundoff aside).
if sat_gap(P,Q)<=0,d=0;return;end
d=inf;
for order=1:2
 if order==1,A=P;B=Q;else,A=Q;B=P;end
 for i=1:size(A,1)
  for j=1:size(B,1)
   a=B(j,:);e=B(mod(j,size(B,1))+1,:)-a;
   t=max(0,min(1,dot(A(i,:)-a,e)/dot(e,e)));
   d=min(d,norm(A(i,:)-a-t*e));
  end
 end
end
end
