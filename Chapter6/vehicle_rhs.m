function derivative=vehicle_rhs(state,input,wheelbase)
derivative=[state(4,:).*cos(state(3,:));state(4,:).*sin(state(3,:));state(4,:).*tan(state(5,:))/wheelbase;state(6,:);input(2,:);input(1,:)];
end
