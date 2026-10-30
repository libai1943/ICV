function V=rectangle_vertices(pose,g)
% Counterclockwise physical footprint, rear-axle reference [x;y;theta].
R=[cos(pose(3)),-sin(pose(3));sin(pose(3)),cos(pose(3))];
V=(R*[-g.LR,g.LW+g.LF,g.LW+g.LF,-g.LR;-g.LB/2,-g.LB/2,g.LB/2,g.LB/2]+pose(1:2))';
end
