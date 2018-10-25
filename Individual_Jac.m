function Individual_Jac(L_i, q, qdot, qddot)

%% This function is used to retrieve out the important terms need to evaluate the equation of motion of the robot
% d/dt(pL_i/pqdot) - pL_i/pq = D_i(q) qddot + C(q, qdot)

load('x_t.mat')
load('xdot_t.mat')
load('xddot_t.mat')

dpL_ipqdot_dt = jacobian(jacobian(L_i, qdot), [q',qdot']') * [qdot',qddot']';
pL_i_pq = jacobian(L_i, q)';
eqn = dpL_ipqdot_dt - pL_i_pq;

D_q = jacobian(eqn, qddot);

% D_q = simplify(jacobian(eqn, qddot));
C_q_qdot = eqn - D_q * qddot;
% C_q_qdot = simplify(C_q_qdot);

D_q_t = subs(D_q,q',x_t');
C_q_qdot_t = subs(C_q_qdot,q',x_t');
C_q_qdot_t = subs(C_q_qdot_t,qdot',xdot_t');
C_q_qdot_t = simplify(C_q_qdot_t,'Steps',1);

ccode(D_q_t)

ccode(C_q_qdot_t)

end

