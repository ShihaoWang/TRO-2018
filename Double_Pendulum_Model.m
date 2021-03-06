function Double_Pendulum_Model()

% This function is used to derive the equation of motion for the double
% pendulum model where the second link is at the center of the first link
syms theta_a theta_adot theta_addot theta_b theta_bdot theta_bddot r rdot rddot real
syms r g m I tau fk real

q = [theta_a r, theta_b]';
qdot = [theta_adot, rdot, theta_bdot]';
qddot = [theta_addot, rddot, theta_bddot]';

rG = [r * cos(theta_a), r * sin(theta_a)]';
vG = jacobian(rG, q) * qdot;

Ek = 1/2 * m * dot(vG, vG) + 1/2 * I * (theta_adot + theta_bdot)^2;
Ep = m * g * r * sin(theta_a);
L = Ek - Ep;
pLpq = jacobian(L, q)';
pLpqdot= jacobian(L, qdot)';
dpLpqdotdt = jacobian(pLpqdot, [q',qdot']') * [qdot', qddot']';
eqn = dpLpqdotdt - pLpq - [0, fk, tau]';
A = simplify(jacobian(eqn, qddot));
B = eqn - A * qddot;
B = simplify(B);
x = -simplify(A\B)


end

