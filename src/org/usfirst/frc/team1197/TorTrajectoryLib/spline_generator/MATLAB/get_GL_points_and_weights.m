function [z,w] = get_GL_points_and_weights(a,b,N)
%
% This function returns the Gauss-Legendre points and integration weights
% in [a,b].
%
% Input a,b -> endpoints of the interval [a,b]
%         N -> number of GL points in [a,b]
%
% Attribution: I wrote this following what professor Daniele Venturi was
% showing us in a discussion section for AMS 147 at UC Santa Cruz when we
% studied numerical integration (Winter quarter 2017).

syms x;

eta = vpasolve(legendreP(N,x) == 0); % zeros in [-1, 1]
dL = diff(legendreP(N,x),x,1); % symbolic derivative of Legendre polynomial

z = eta*(b-a)/2 + (b+a)/2; % zeros in [a, b]
w = (b-a)./((1-eta.^2).*subs(dL, eta).^2);

end
