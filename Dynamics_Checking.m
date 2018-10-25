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

% Body
[rLeftBody,vLeftBody,omegaBody,IBody,RBody] = autoGen_Body(State)

% Left Shoulder
[rLeftShoulder,vLeftShoulder,omegaLeftShoulder,ILeftShoulder,RLeftShoulder] = autoGen_LeftShoulder(State)
% Left Arm
[rLeftArm,vLeftArm,omegaLeftArm,ILeftArm,RLeftArm] = autoGen_LeftArm(State)
% Left Hand
[rLeftHand,vLeftHand,omegaLeftHand,ILeftHand,RLeftHand] = autoGen_LeftHand(State)

% Right Hip
[rRightHip,vRightHip,omegaRightHip,IRightHip,RRightHip] = autoGen_RightHip(State)
% Right Thigh
[rRightThigh,vRightThigh,omegaRightThigh,IRightThigh,RRightThigh] = autoGen_RightThigh(State)
% Right Shank
[rRightShank,vRightShank,omegaRightShank,IRightShank,RRightShank] = autoGen_RightShank(State)
% Right Ankle
[rRightAnkle,vRightAnkle,omegaRightAnkle,IRightAnkle,RRightAnkle] = autoGen_RightAnkle(State)
% Right Foot
[rRightFoot,vRightFoot,omegaRightFoot,IRightFoot,RRightFoot] = autoGen_RightFoot(State)

% Right Shoulder
[rRightShoulder,vRightShoulder,omegaRightShoulder,IRightShoulder,RRightShoulder] = autoGen_RightShoulder(State)
% Right Arm
[rRightArm,vRightArm,omegaRightArm,IRightArm,RRightArm] = autoGen_RightArm(State)
% Right Hand
[rRightHand,vRightHand,omegaRightHand,IRightHand,RRightHand] = autoGen_RightHand(State)

% Left Hip
[rLeftHip,vLeftHip,omegaLeftHip,ILeftHip,RLeftHip] = autoGen_LeftHip(State)
% Left Thigh
[rLeftThigh,vLeftThigh,omegaLeftThigh,ILeftThigh,RLeftThigh] = autoGen_LeftThigh(State)
% Left Shank
[rLeftShank,vLeftShank,omegaLeftShank,ILeftShank,RLeftShank] = autoGen_LeftShank(State)
% Left Ankle
[rLeftAnkle,vLeftAnkle,omegaLeftAnkle,ILeftAnkle,RLeftAnkle] = autoGen_LeftAnkle(State)
% Left Foot
[rLeftFoot,vLeftFoot,omegaLeftFoot,ILeftFoot,RLeftFoot] = autoGen_LeftFoot(State)

end

