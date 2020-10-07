% function b = Newton(FunFcn,x,p)
function b = fzerom_par(FunFcn,x,p)
%FZEROS  Solves set of functions (linear or non linear) by means of
% Newton-Raphson iteration. 
% FunFcn is a string holding name of function
% x is initial guess on variables
% p is a set of fixed parameters
% tol is the tolerance to be met in iteration
% 
% Michael Rygaard Hansen
% Aalborg University, 1999
%
% Initialization
if nargin < 4 | isempty(tol), tol = 1e-10; end
if nargin < 5 | isempty(relax), relax = 1.0; end

% Set tolerance
% tol = 1e-5;

%set relaxation
relax = 1.0;

% Get rank of problem
n = length(x);

% Set perturbation
pert = 1e-10;

% Initialize counter
i = 0;

% Initial residual vector computation
b = feval(FunFcn, x, p);
n2 = length(b);

% Do Newton-Raphson iteration procedure. Stop when counter = 100 or
% tolerance met.
while norm(b)>tol & i<100

  % Increment counter
  i = i+1;

  % Generate jacobian
  for j=1:n
    x(j) = x(j)+pert;
    bPert = feval(FunFcn, x, p);
    x(j) = x(j)-pert;
    for k=1:n
      jacob(k,j)=(bPert(k)-b(k))/pert;
    end
 end  
  
  % Do correction
  x = x - relax*(jacob\b);
  b = feval(FunFcn, x, p);
  c = norm(b);
end
b = x;
