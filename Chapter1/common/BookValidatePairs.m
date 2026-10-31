function metrics = BookValidatePairs(book)
%BOOKVALIDATEPAIRS Independent oriented-rectangle separation and interval bound.
% The interval certificate applies to piecewise-linear rear-axle positions and
% unwrapped headings, not to an untested continuous-time integration method.
q=book.state;t=book.time;nv=size(q,3);nk=numel(t);
assert(all(isfinite(q(:))) && all(diff(t)>0),'Book:InvalidData','Nonfinite states or invalid time vector.');
minimum=inf(1,nk);interval=inf(1,nk-1);
for i=1:nv
    g=book.geometry(i);a=(g.LW+g.LF+g.LR)/2;b=g.LB/2;offset=a-g.LR;
    hi=q(3,:,i);ci=cos(hi);si=sin(hi);
    xi=q(1,:,i)+offset*ci;yi=q(2,:,i)+offset*si;
    boundi=hypot(diff(q(1,:,i)),diff(q(2,:,i)))+hypot(max(g.LR,g.LW+g.LF),b)*abs(diff(hi));
    for j=i+1:nv
        h=book.geometry(j);aj=(h.LW+h.LF+h.LR)/2;bj=h.LB/2;offj=aj-h.LR;
        hj=q(3,:,j);cj=cos(hj);sj=sin(hj);
        dx=q(1,:,j)+offj*cj-xi;dy=q(2,:,j)+offj*sj-yi;
        c=abs(cos(hj-hi));s=abs(sin(hj-hi));
        gaps=[abs(dx.*ci+dy.*si)-a-aj*c-bj*s; ...
            abs(-dx.*si+dy.*ci)-b-aj*s-bj*c; ...
            abs(dx.*cj+dy.*sj)-aj-a*c-b*s; ...
            abs(-dx.*sj+dy.*cj)-bj-a*s-b*c];
        sep=max(gaps,[],1);minimum=min(minimum,sep);
        boundj=hypot(diff(q(1,:,j)),diff(q(2,:,j)))+hypot(max(h.LR,h.LW+h.LF),bj)*abs(diff(hj));
        lower=min(sep(1:end-1),sep(2:end))-0.5*(boundi+boundj);
        interval=min(interval,lower);
    end
end
metrics.minimum_sampled_separating_axis_gap=min(minimum);
metrics.minimum_interval_clearance_bound=min(interval);
metrics.sampled_rectangles_disjoint=all(minimum>=-1e-7);
metrics.linear_pose_interpolation_certified=all(interval>=0);
metrics.interpolation='piecewise-linear rear-axle position and unwrapped heading';
metrics.sampled_gap=minimum;metrics.interval_bound=interval;
end
