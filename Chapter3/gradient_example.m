function result=gradient_example(out)
E=@(x)exp(-.1*x.^2);f=@(x)tanh(x)+.2*x.*E(x)+.3*sin(2.5*x);
df=@(x)sech(x).^2+.2*(1-.2*x.^2).*E(x)+.75*cos(2.5*x);
starts=[-6,-3.2,-1,-.2,.05,.8,1.8,2.3,3.14,3.5,4.2,5.5];result=zeros(numel(starts),4);
for j=1:numel(starts)
 x=starts(j);
 for k=1:500
  next=x-.1*df(x);stop=abs(next-x)<1e-4;x=next;if stop,break;end
 end
 assert(stop);result(j,:)=[starts(j),x,f(x),k];
end
writetable(array2table(result,'VariableNames',{'initial_x','terminal_x','cost','updates'}),fullfile(out,'gradient_results.csv'));
end
