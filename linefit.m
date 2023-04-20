function x = linefit(y,bp)

N = size(y,1);

bp = unique([0;bp(:);N-1]);	% Include both endpoints
lb = length(bp)-1;
a  = [zeros(N,lb) ones(N,1)];	% Build regressor with linear pieces + DC
for kb = 1:lb
  M = N - bp(kb);
  a([1:M]+bp(kb),kb) = [1:M]'/M;
end
x = a*(a\y);		% best fit