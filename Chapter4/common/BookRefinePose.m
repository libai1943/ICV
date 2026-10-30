function fine=BookRefinePose(book,factor)
%BOOKREFINEPOSE Preserve every original knot in the same linear-pose trajectory.
assert(factor>=1 && factor==round(factor));fine=book;t=book.time;fine.time=[];
for k=1:numel(t)-1
 segment=linspace(t(k),t(k+1),factor+1);fine.time=[fine.time,segment(1:end-1)]; %#ok<AGROW>
end
fine.time=[fine.time,t(end)];fine.state=zeros(5,numel(fine.time),size(book.state,3));
for i=1:size(book.state,3)
 q=book.state(:,:,i);q(3,:)=unwrap(q(3,:));
 fine.state(:,:,i)=interp1(t,q',fine.time,'linear')';
end
end
