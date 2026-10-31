function centers=disc_centers(state,geometry)
length=geometry.LW+geometry.LF+geometry.LR;
offset=[-geometry.LR+length/4,-geometry.LR+3*length/4];
centers=cell(1,2);
for side=1:2,centers{side}=state(1:2,:)+offset(side)*[cos(state(3,:));sin(state(3,:))];end
end
