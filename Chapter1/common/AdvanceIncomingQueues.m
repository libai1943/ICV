function book=AdvanceIncomingQueues(book)
% Advance queued vehicles as their predecessor clears space, then meet the
% reserved maneuver entry state. Each approach is a convex longitudinal OCP.
import casadi.*
dt=book.time(2)-book.time(1);nv=size(book.state,3);
for i=1:nv
    K=round(book.departure_times(i)/dt)+1;
    heading=book.state(3,1,i);direction=[cos(heading);sin(heading)];
    q=book.state(:,1:K,i);position=direction'*q(1:2,:);
    problem=Opti();x=problem.variable(1,K);v=problem.variable(1,K);a=problem.variable(1,K);
    problem.set_initial(x,position);problem.set_initial(v,q(4,:));problem.set_initial(a,book.control(1,1:K,i));
    problem.subject_to(x(1)==position(1));problem.subject_to(x(end)==position(end));
    problem.subject_to(v(1)==q(4,1));problem.subject_to(v(end)==q(4,end));
    problem.subject_to(a(end)==0);problem.subject_to(v>=0);problem.subject_to(v<=2);
    problem.subject_to(a>=-1.2);problem.subject_to(a<=1.2);
    problem.subject_to(x(2:end)==x(1:end-1)+dt*v(1:end-1));
    problem.subject_to(v(2:end)==v(1:end-1)+dt*a(1:end-1));
    problem.subject_to(x<=position(end));
    if i>4
        front=direction'*book.state(1:2,1:K,i-4);
        problem.subject_to(x<=front-5.7);
    end
    problem.minimize(-sum2(x)+.02*sumsqr(a));
    ip=struct('print_level',0,'sb','yes','tol',1e-9,'max_iter',1500,'linear_solver','ma57','bound_relax_factor',0);
    ip.hsllib=getenv('IPOPT_HSL_LIBRARY');problem.solver('ipopt',struct('print_time',false),ip);
    sol=problem.solve();distance=sol.value(x)-position;
    book.state(1:2,1:K,i)=q(1:2,:)+direction*distance;
    book.state(4,1:K,i)=sol.value(v);book.control(1,1:K,i)=sol.value(a);
    fprintf('Queue advancement %d/%d\n',i,nv);
end
book.validation=BookValidatePairs(BookRefinePose(book,20));
assert(book.validation.linear_pose_interpolation_certified,'Book:Queues','Advanced queues require further collision resolution.');
end
