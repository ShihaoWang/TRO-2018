function omega = EulerAngNVel2Omega(Euler_Ang, Euler_Vel)

% This function transfer the aircraft Euler angle/velocity to actual
% angular velocity

%% Based on the default specification, the Euler_Ang is [Yaw, Pitch, Roll] and Euler_Vel is [Yaw', Pitch' and Roll']
% According to the standard notation, the Yaw: psai, Pitch: theta, Roll: fai

psai = Euler_Ang(1);
theta = Euler_Ang(2);
fai = Euler_Ang(3);

psaidot = Euler_Vel(1);
thetadot = Euler_Vel(2);
faidot = Euler_Vel(3);

%% Then the ifnal angular velocity in the local body frame is

omega_x = faidot - psaidot * sin(theta);
omega_y = psaidot * cos(theta) * sin(fai) + thetadot * cos(fai);
omega_z = psaidot * cos(theta) * cos(fai) - thetadot * sin(fai);

omega = [omega_x, omega_y, omega_z]';

end

