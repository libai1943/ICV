function [radius,offsets]=disc_geometry(scene)
% Equal longitudinal slabs: each disc covers the complete width of its slab.
g=scene.geometry;length=g.LW+g.LF+g.LR;n=4;
radius=hypot(length/n,g.LB)/2;
offsets=-g.LR+length/n*((1:n)-.5);
end
