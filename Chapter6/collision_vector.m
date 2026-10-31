function [distance,radius]=collision_vector(state,task)
% Each column is one disc-disc or disc-obstacle interaction; rows are times.
count=size(state,3);centers=cell(count,2);blocks={};radius=[];
for car=1:count,centers(car,:)=disc_centers(state(:,:,car),task.geometry);end
cover=hypot((task.geometry.LW+task.geometry.LF+task.geometry.LR)/4,task.geometry.LB/2);
for car=1:count
 for other=car+1:count
  for a=1:2,for b=1:2
   delta=centers{car,a}-centers{other,b};blocks{end+1}=sum(delta.^2,1)';radius(end+1)=2*cover;
  end,end
 end
 for obstacle=1:size(task.obstacles,1)
  for a=1:2
   delta=centers{car,a}-task.obstacles(obstacle,1:2)';blocks{end+1}=sum(delta.^2,1)';radius(end+1)=cover+task.obstacles(obstacle,3);
  end
 end
end
distance=horzcat(blocks{:});
end
