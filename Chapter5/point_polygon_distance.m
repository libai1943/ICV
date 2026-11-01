function distance=point_polygon_distance(points,polygon)
% Vectorized distance from planar points to the occupied polygon.
distance=inf(size(points,1),1);ends=polygon([2:end,1],:);
for k=1:size(polygon,1)
 edge=ends(k,:)-polygon(k,:);delta=points-polygon(k,:);
 t=max(0,min(1,(delta*edge')/max(sum(edge.^2),1e-20)));
 distance=min(distance,vecnorm(delta-t.*edge,2,2));
end
distance(inpolygon(points(:,1),points(:,2),polygon(:,1),polygon(:,2)))=0;
end
