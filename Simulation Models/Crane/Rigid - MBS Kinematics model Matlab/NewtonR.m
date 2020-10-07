function b = NewtonR(Phi,Jacobi,x,t,eps,relax)
% function b = NewtonR(Phi,Jacobi,x,t,eps,k1,k2)
%
% Phi er en streng indeholdende navnet på funktionen 
%     indeholdende bindingsligningerne Phi(x,p)
% Jacobi er en streng indeholdende navnet på funktionen
%     indeholdende jacobimatrice Jacobi(x,p)
% x er en vektor med startgættet for de variable koordinater
% p er en vektor med mulige konstanter. Hvis ingen konstanter 
%     eksisterer sættes p til et 0 i funktionskaldet
% relax er en relaxationsfaktor for ændringen af koordinatvektoren
%     i Newton-Raphson iterationen. Den bør som standard sættes 
%     til 1 i funktionskaldet.
% 
% Output fra NewtonR er en vektor med koordinater, der tilfredsstiller 
% det opstillede ligningssystem
% 
% 
% Morten Ebbesen, Aalborg University, 2005

if nargin < 5 | isempty(tol), tol = 1e-10; end
if nargin < 6 | isempty(relax), relax = 1.0; end

nmax = 150;
% error = norm(feval(Phi,x,t,k1,k2));
error = norm(feval(Phi,x,t));
i = 0;
while (error>tol & i<nmax)
    i = i + 1;
%     deltax = feval(Jacobi,x,k1)\feval(Phi,x,t,k1,k2);
    J = feval(Jacobi,x);
    P = feval(Phi,x,t);
    deltax = feval(Jacobi,x)\feval(Phi,x,t);
    error = norm(deltax);
    x = x - relax*deltax;
%     x = x - deltax;
end
b = x;