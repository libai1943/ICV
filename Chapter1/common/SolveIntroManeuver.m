function result = SolveIntroManeuver(scene, reference)
%SOLVEINTROMANEUVER Direct Euler transcription with hard kinematic constraints.
% State: [x; y; theta; v; phi]. Control: [a; omega]. Units: m, s, rad.
% Two overlapping covering discs use the upstream STC constructor. This
% single-vehicle solve is used only for the introductory illustration.
import casadi.*
parameters = scene.parameters;
vehicle = scene.vehicles;
nodeCount = parameters.nodes;
bodyLength = vehicle.LW + vehicle.LF + vehicle.LR;
coverOffsets = -vehicle.LR + bodyLength * [0.25, 0.75];
coverRadius = hypot(bodyLength/4, vehicle.LB/2);
boxes = cell(1,2);
for disc = 1:2
    proxy = scene;
    proxy.vehicles.radius = coverRadius + 0.08;
    seed = reference;
    seed(8:9,:) = seed(1:2,:) + coverOffsets(disc)*[cos(seed(7,:));sin(seed(7,:))];
    corridors = BuildCorridors(proxy, seed);
    boxes{disc} = corridors.absolute;
end
problem = Opti();
state = problem.variable(5, nodeCount);
control = problem.variable(2, nodeCount);
problem.set_initial(state, reference([1, 2, 7, 3, 5], :));
problem.set_initial(control, reference([4, 6], :));

% Prescribed initial and terminal states, including the prescribed outbound speed.
problem.subject_to(state(:, 1) == vehicle.initial([1, 2, 7, 3, 5]));
problem.subject_to(state(:, end) == [vehicle.desired; vehicle.heading; vehicle.exit_speed; 0]);
problem.subject_to(control(:, 1) == 0);
problem.subject_to(control(:, end) == 0);
problem.subject_to(state(4, :) >= 0);
problem.subject_to(state(4, :) <= parameters.vmax);
problem.subject_to(state(5, :) >= -parameters.phimax);
problem.subject_to(state(5, :) <= parameters.phimax);
problem.subject_to(control(1, :) >= -parameters.amax);
problem.subject_to(control(1, :) <= parameters.amax);
problem.subject_to(control(2, :) >= -parameters.omegamax);
problem.subject_to(control(2, :) <= parameters.omegamax);

% Euler defects are equality constraints, rather than a soft cost penalty.
leftState = state(:, 1:end-1);
leftControl = control(:, 1:end-1);
rate = [leftState(4, :) .* cos(leftState(3, :));
        leftState(4, :) .* sin(leftState(3, :));
        leftState(4, :) .* tan(leftState(5, :)) / vehicle.LW;
        leftControl(1, :);
        leftControl(2, :)];
problem.subject_to(state(:, 2:end) == leftState + parameters.dt * rate);
center = [state(1, :) + vehicle.offset * cos(state(3, :));
          state(2, :) + vehicle.offset * sin(state(3, :))];
for disc = 1:2
    discCenter = [state(1,:) + coverOffsets(disc)*cos(state(3,:));
                  state(2,:) + coverOffsets(disc)*sin(state(3,:))];
    lower = boxes{disc}([1,3],:);
    upper = boxes{disc}([2,4],:);
    problem.subject_to(discCenter(:) >= lower(:));
    problem.subject_to(discCenter(:) <= upper(:));
end
problem.minimize(0.01 * parameters.dt * sumsqr(control));

options = struct('print_time', false);
ipopt = struct('print_level', 0, 'sb', 'yes', 'tol', 1e-8, ...
    'acceptable_iter', 0, 'max_iter', 2500, 'max_cpu_time', 120, ...
    'linear_solver', 'ma57', 'bound_relax_factor', 0);
ipopt.hsllib = getenv('IPOPT_HSL_LIBRARY');
problem.solver('ipopt', options, ipopt);
solution = problem.solve();
stateValue = solution.value(state);
controlValue = solution.value(control);
% Preserve the upstream nine-row format at this adapter boundary.
fleet = [stateValue(1:2, :); stateValue(4, :); controlValue(1, :);
         stateValue(5, :); controlValue(2, :); stateValue(3, :);
         solution.value(center)];
result.fleet = fleet;
result.metrics = ValidateSolution(scene, fleet, false);
result.metrics.single_disc_clearance_not_used = result.metrics.minimum_clearance;
clearance = inf;
for disc = 1:2
    c = fleet(1:2,:) + coverOffsets(disc)*[cos(fleet(7,:));sin(fleet(7,:))];
    for obstacle = scene.obstacles'
        dx = max(max(obstacle(1)-c(1,:), c(1,:)-obstacle(2)),0);
        dy = max(max(obstacle(3)-c(2,:), c(2,:)-obstacle(4)),0);
        clearance = min(clearance,min(hypot(dx,dy)-coverRadius));
    end
end
result.metrics.minimum_clearance = clearance;
result.metrics = rmfield(result.metrics, {'clearance','pair_clearance'});
result.metrics.collision_model = 'two overlapping discs covering the full rectangle';
assert(clearance >= -1e-6 && result.metrics.max_defect < 1e-6 && result.metrics.hard_violation < 1e-6);

fprintf('Hard-constraint maneuver: defect %.3g, clearance %.3g m\n', ...
    result.metrics.max_defect, result.metrics.minimum_clearance);
end
