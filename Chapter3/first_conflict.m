function conflict=first_conflict(paths)
% Complete vertex and edge-swap checks for this four-connected point model.
conflict=[];T=max(cellfun(@(p)size(p,1),paths));
for k=1:T
 for i=1:numel(paths)-1
  p=paths{i};a=p(min(k,end),:);a1=p(min(k+1,end),:);
  for j=i+1:numel(paths)
   q=paths{j};b=q(min(k,end),:);b1=q(min(k+1,end),:);
   if isequal(a,b),conflict=struct('type','vertex','i',i,'j',j,'time',k-1,'a',a,'b',b,'a1',a1,'b1',b1);return;end
   if isequal(a,b1)&&isequal(b,a1),conflict=struct('type','edge','i',i,'j',j,'time',k-1,'a',a,'b',b,'a1',a1,'b1',b1);return;end
  end
 end
end
end
