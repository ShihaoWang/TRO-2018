function AMPM_Dyn()

% This function is used to derive the equation of motion for the angular
% momentum based inverted pendulum model
syms theta_a theta_adot theta_addot theta_b theta_bdot theta_bddot real
syms r rdot rddot m I g tau f real

vG = [rdot, r * theta_adot]';
Ek = 1/2 * m * dot(vG, vG) + 1/2 * I * (theta_bdot) * (theta_bdot);
Ep = m * g * r * sin(theta_a);

L = Ek - Ep;

q = [theta_a, r, theta_b]';
qdot = [theta_adot, rdot, theta_bdot]';
qddot = [theta_addot, rddot, theta_bddot]';

pLpq = jacobian(L , q);
pLpqdot = jacobian(L, qdot);
dpLpdotdt = simplify(jacobian(pLpqdot', [q', qdot']') * [qdot', qddot']');

eqn = -pLpq' + dpLpdotdt - [0, f, tau]';

A = jacobian(eqn, qddot);
B = simplify(A * qddot - eqn);
acc = simplify(A\B)

% eqn1 = cos(theta_a) * rddot - r * sin(theta_a) * theta_addot - r* theta_adot^2 * cos(theta_adot) - 2 * rdot * theta_adot *



end

