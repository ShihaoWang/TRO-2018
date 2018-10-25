function Dynamics_Checking()

% This function is used to check the dynamics between the Python version
% and the Matlab version

Config = load('Init_Config.txt');
Velocity = load('Init_Velocity.txt');
State = [Config; Velocity];
load('p.mat')

%% The list of the comparable variables
% 1. Global COM position
% 2. Local COM position
% 3. COM velocity
% 4. Angular velocity of a certain link
% 5. Moment of Inertia of this link in local frame




end

