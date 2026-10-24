function [x,request,delta] = reference_loop(c,M,K)
% Base-MATLAB reference for checking the sampled Simulink implementation.
N=numel(c.t); x=zeros(N,4); x(1,:)=c.x0';
request=zeros(N,1); delta=zeros(N,1); previous=0;
for i=1:N
    if c.tracking
        request(i)=c.ff(i)+K*(c.xref(i,:)-x(i,:))';
    else
        request(i)=c.open(i);
    end
    bounded=min(max(request(i),-c.limit),c.limit);
    increment=min(max(bounded-previous,-c.rate*c.Ts),c.rate*c.Ts);
    delta(i)=previous+increment;
    if i<N, x(i+1,:)=(M.Ad*x(i,:)'+M.Bd*delta(i))'; end
    previous=delta(i);
end
end
