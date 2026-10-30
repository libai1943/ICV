function [gap,axis]=sat_gap(P,Q)
% Maximum signed projection gap over the edge normals of both polygons.
% Positive: separated; zero: touching; negative: overlapping interiors.
% A positive gap lower-bounds Euclidean clearance; it need not equal it.
gap=-inf;axis=[nan,nan];
for shape={P,Q}
 V=shape{1};E=V([2:end,1],:)-V;
 for k=1:size(E,1)
  n=[-E(k,2),E(k,1)];n=n/norm(n);a=P*n';b=Q*n';
  candidates=[min(b)-max(a),min(a)-max(b)];[v,side]=max(candidates);
  if v>gap,gap=v;axis=n*(3-2*side);end
 end
end
end
