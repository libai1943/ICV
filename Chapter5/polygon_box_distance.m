function distance=polygon_box_distance(box,polygon)
% Euclidean polygon-to-box distance, including edge crossings.
epsilon=1e-10;box=box+[-epsilon,epsilon,-epsilon,epsilon];
corners=[box(1),box(3);box(2),box(3);box(2),box(4);box(1),box(4)];
% Four-point ray crossing avoids repeated general-purpose polygon preprocessing.
ends=polygon([2:end,1],:);dy=ends(:,2)-polygon(:,2);dy(abs(dy)<1e-30)=1;
crosses=(polygon(:,2)>corners(:,2)')~=(ends(:,2)>corners(:,2)');
intersection=polygon(:,1)+(ends(:,1)-polygon(:,1)).*(corners(:,2)'-polygon(:,2))./dy;
inside=mod(sum(crosses&(corners(:,1)'<intersection),1),2)==1;
if any(inside)||any(polygon(:,1)>=box(1)&polygon(:,1)<=box(2)&polygon(:,2)>=box(3)&polygon(:,2)<=box(4))
 distance=0;return
end
next=polygon([2:end,1],:);edge=next-polygon;
lo=zeros(size(polygon,1),1);hi=ones(size(lo));valid=true(size(lo));
for axis=1:2
 low=box(2*axis-1);high=box(2*axis);flat=abs(edge(:,axis))<1e-12;
 valid=valid&~(flat&(polygon(:,axis)<low|polygon(:,axis)>high));
 divisor=edge(:,axis);divisor(flat)=1;
 a=(low-polygon(:,axis))./divisor;b=(high-polygon(:,axis))./divisor;
 a(flat)=-inf;b(flat)=inf;lo=max(lo,min(a,b));hi=min(hi,max(a,b));
end
if any(valid&lo<=hi),distance=0;return;end
dx=max(max(box(1)-polygon(:,1),polygon(:,1)-box(2)),0);dy=max(max(box(3)-polygon(:,2),polygon(:,2)-box(4)),0);distance=min(hypot(dx,dy));
for k=1:4
 projection=max(0,min(1,sum((corners(k,:)-polygon).*edge,2)./sum(edge.^2,2)));
 distance=min(distance,min(vecnorm(corners(k,:)-polygon-projection.*edge,2,2)));
end
end
