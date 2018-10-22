function Variable_Ranger(State, p)

% This function is used to enumerate the range of the variable given the
% different robot state 

H_COM = H_COM_fn(State, p);

I_COM = I_COM_fn(State,p);

%% The average angular velocity in the x direction
omega_x = H_COM(1)/I_COM(1,1);
omega_y = H_COM(2)/I_COM(2,2);
omega_z = H_COM(3)/I_COM(3,3);




end

