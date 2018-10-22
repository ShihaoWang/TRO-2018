function RMP_Dynamics_Deri()

% This function is used to derive the equation of motion of the reactive
% mass pendulum (RMP)
alpha = [];
syms theta L r alpha thetadot Ldot rdot alphadot thetaddot Lddot rddot alphaddot g m real
syms fL fr tau real

rO = [ 0 0]';
rOM = [ L * cos(theta), L * sin(theta)]';
rM = rO + rOM;
rMB = [r * cos(alpha + theta - pi), r * sin(alpha + theta - pi)]';
rMA = -rMB;
rA = rM + rMA;
rB = rM + rMB;

x = [theta, L, r alpha]';
xdot = [thetadot, Ldot, rdot, alphadot]';
xddot = [thetaddot, Lddot, rddot, alphaddot]';

vA = jacobian(rA, x) * xdot;
vB = jacobian(rB, x) * xdot;

% Kinetic energy

Ek_A = 1/2 * 1/2 * m * dot(vA, vA);
Ek_B = 1/2 * 1/2 * m * dot(vB, vB);

Ep = m * g * L * sin(theta);

L = Ek_A + Ek_B + Ep;
L = simplify(L);

pLpx = jacobian(L, x)';
pLpxdot = jacobian(L, xdot);
dpLpxdotdt = jacobian(pLpxdot, [x',xdot']') * [xdot', xddot']';

eqn = dpLpxdotdt - pLpx - [0; fL; fr; tau];
eqn = simplify(eqn);
A = jacobian(eqn, xddot);
B = A * xddot - eqn;
B = simplify(B);
xddot_sym = simplify(A\B)

syms theta(t) L 

ode = diff(diff(theta,t),t) == g / L * cos(theta) 
ySol(t) = dsolve(ode)

eqn = m * L^2 * thetaddot - m * g * L * cos(theta)

end

