function value=area_residual(p,P)
% Triangle-area sum minus polygon area; point membership, not polygon SAT.
Q=P([2:end,1],:);a=P-p;b=Q-p;
value=.5*sum(abs(a(:,1).*b(:,2)-a(:,2).*b(:,1)))-polyarea(P(:,1),P(:,2));
end
