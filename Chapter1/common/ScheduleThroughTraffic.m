function book = ScheduleThroughTraffic(scene, paths)
%SCHEDULETHROUGHTRAFFIC Select collision-free departures for prescribed maneuvers.
% Vehicles queue with physical spacing, then accelerate onto their maneuvers.
% Departure continuations remain in the global collision checks.
% Coordinate arrival times by a fixed-priority search. All outgoing traffic
% remains in the collision checks. This is not a time-optimality claim.
timeStep = scene.parameters.dt;
pathNodes = scene.parameters.nodes;
vehicleCount = numel(paths);
scheduleNodes = 1801;
offset = zeros(1, vehicleCount);
fleet = zeros(9, scheduleNodes, vehicleCount);
approach=cell(1,vehicleCount);approachSteps=zeros(1,vehicleCount);
for i = 1:vehicleCount
    q=paths{i}(:,1);speed=q(3);nacc=15;
    acceleration=speed/(nacc*timeStep);
    accelDistance=speed*timeStep*(nacc-1)/2;
    desiredDistance=6*(floor((i-1)/4)+1);
    ncruise=round((desiredDistance-accelDistance)/(speed*timeStep));
    velocity=[(0:nacc)*speed/nacc,repmat(speed,1,ncruise)];
    distance=[0,cumsum(velocity(1:end-1))*timeStep];
    pre=repmat(q,1,numel(velocity));
    pre(3,:)=velocity;pre(4,:)=0;pre(4,1:nacc)=acceleration;
    displacement=distance-distance(end);
    pre(1,:)=q(1)+displacement*cos(q(7));pre(2,:)=q(2)+displacement*sin(q(7));
    pre(8,:)=q(8)+displacement*cos(q(7));pre(9,:)=q(9)+displacement*sin(q(7));
    approach{i}=pre;approachSteps(i)=size(pre,2)-1;
    parked=pre(:,1);parked(4)=0;
    fleet(:,:,i)=repmat(parked,1,scheduleNodes);
end
for i = 1:vehicleCount
    found = false;
    for shift = approachSteps(i):4:scheduleNodes-pathNodes-90
        parked=approach{i}(:,1);parked(4)=0;
        trial=repmat(parked,1,scheduleNodes);
        first=shift-approachSteps(i)+1;
        trial(:,first:shift+1)=approach{i};
        trial(:,shift+(1:pathNodes))=paths{i};
        late=shift+pathNodes+1:scheduleNodes;
        q=paths{i}(:,end);t=(1:numel(late))*timeStep;
        trial(:,late)=repmat(q,1,numel(late));
        dx=q(3)*cos(q(7))*t;dy=q(3)*sin(q(7))*t;
        trial(1,late)=q(1)+dx;trial(2,late)=q(2)+dy;
        trial(8,late)=q(8)+dx;trial(9,late)=q(9)+dy;
        admissible = true;
        for j = setdiff(1:vehicleCount,i)
            gi = scene.vehicles(i); gj = scene.vehicles(j);
            ai = (gi.LW+gi.LF+gi.LR)/2; bi = gi.LB/2;
            aj = (gj.LW+gj.LF+gj.LR)/2; bj = gj.LB/2;
            hi = trial(7,:); hj = fleet(7,:,j);
            xi = trial(1,:) + (ai-gi.LR)*cos(hi);
            yi = trial(2,:) + (ai-gi.LR)*sin(hi);
            dx = fleet(1,:,j)+(aj-gj.LR)*cos(hj)-xi;
            dy = fleet(2,:,j)+(aj-gj.LR)*sin(hj)-yi;
            c = abs(cos(hj-hi)); s = abs(sin(hj-hi));
            gaps = [abs(dx.*cos(hi)+dy.*sin(hi))-ai-aj*c-bj*s;
                abs(-dx.*sin(hi)+dy.*cos(hi))-bi-aj*s-bj*c;
                abs(dx.*cos(hj)+dy.*sin(hj))-aj-ai*c-bi*s;
                abs(-dx.*sin(hj)+dy.*cos(hj))-bj-ai*s-bi*c];
            if min(max(gaps,[],1)) < 0.4
                admissible = false;
                break;
            end
        end
        if admissible
            found = true;
            offset(i) = shift;
            fleet(:, :, i) = trial;
            fprintf('Vehicle %d departure %.2f s\n', i, shift*timeStep);
            break;
        end
    end
    assert(found, 'Book:Schedule', 'No collision-free departure found.');
end
lastNode = max(offset) + pathNodes + 90;
fleet = fleet(:, 1:lastNode, :);
book.time = (0:lastNode-1) * timeStep;
book.state = fleet([1, 2, 7, 3, 5], :, :);
book.control = fleet([4, 6], :, :);
book.geometry = scene.vehicles;
book.scene = scene;
book.departure_times = offset * timeStep;
book.approach_steps=approachSteps;
book.validation = BookValidatePairs(BookRefinePose(book, 20));
assert(book.validation.linear_pose_interpolation_certified, ...
    'Book:Continuous', 'Vehicle separation not certified.');
end
