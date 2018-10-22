function Average_Angular_Velocity()

% This function is used to understand the average angular velocity

syms omega_x omega_y omega_z rk_x rk_y rk_z real


omega = [omega_x, omega_y, omega_z]';
rk = [rk_x, rk_y, rk_z]';

% Now it is the double cross product

KE_Rot = dot(cross(omega,rk), cross(omega, rk));

I_int = jacobian(jacobian(KE_Rot, omega), omega)


end

