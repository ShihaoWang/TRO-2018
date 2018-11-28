function Dynamics_Checking()

% This function is used to check the dynamics between the Python version
% and the Matlab version

format long

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

rOx = State(1);         rOy = State(2);         rOz = State(3);
alpha = State(4);       beta = State(5);        gamma = State(6);
q1 = State(7);          q2 = State(8);          q3 = State(9);
q4 = State(10);         q5 = State(11);         q6 = State(12);
q7 = State(13);         q8 = State(14);         q9 = State(15);
q10 = State(16);        q11 = State(17);        q12 = State(18);
q13 = State(19);        q14 = State(20);        q15 = State(21);
q16 = State(22);    

rOxdot = State(23);     rOydot = State(24);     rOzdot = State(25);
alphadot = State(26);   betadot = State(27);    gammadot = State(28);
q1dot = State(29);      q2dot = State(30);      q3dot = State(31);
q4dot = State(32);      q5dot = State(33);      q6dot = State(34);
q7dot = State(35);      q8dot = State(36);      q9dot = State(37);
q10dot = State(38);     q11dot = State(39);     q12dot = State(40);
q13dot = State(41);     q14dot = State(42);     q15dot = State(43);
q16dot = State(44);

% Body
[rBody,vBody,omegaBody,IBody,RBody] = autoGen_Body(rOx,rOy,rOz,alpha,beta,gamma,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,rOxdot,rOydot,rOzdot,alphadot,betadot,gammadot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot,q11dot,q12dot,q13dot,q14dot,q15dot,q16dot);
% Left Shoulder
[rLeftShoulder,vLeftShoulder,omegaLeftShoulder,ILeftShoulder,RLeftShoulder] = autoGen_LeftShoulder(rOx,rOy,rOz,alpha,beta,gamma,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,rOxdot,rOydot,rOzdot,alphadot,betadot,gammadot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot,q11dot,q12dot,q13dot,q14dot,q15dot,q16dot);
% Left Arm
[rLeftArm,vLeftArm,omegaLeftArm,ILeftArm,RLeftArm] = autoGen_LeftArm(rOx,rOy,rOz,alpha,beta,gamma,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,rOxdot,rOydot,rOzdot,alphadot,betadot,gammadot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot,q11dot,q12dot,q13dot,q14dot,q15dot,q16dot);
% Left Hand
[rLeftHand,vLeftHand,omegaLeftHand,ILeftHand,RLeftHand] = autoGen_LeftHand(rOx,rOy,rOz,alpha,beta,gamma,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,rOxdot,rOydot,rOzdot,alphadot,betadot,gammadot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot,q11dot,q12dot,q13dot,q14dot,q15dot,q16dot);

% Right Hip
[rRightHip,vRightHip,omegaRightHip,IRightHip,RRightHip] = autoGen_RightHip(rOx,rOy,rOz,alpha,beta,gamma,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,rOxdot,rOydot,rOzdot,alphadot,betadot,gammadot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot,q11dot,q12dot,q13dot,q14dot,q15dot,q16dot);
% Right Thigh
[rRightThigh,vRightThigh,omegaRightThigh,IRightThigh,RRightThigh] = autoGen_RightThigh(rOx,rOy,rOz,alpha,beta,gamma,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,rOxdot,rOydot,rOzdot,alphadot,betadot,gammadot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot,q11dot,q12dot,q13dot,q14dot,q15dot,q16dot);
% Right Shank
[rRightShank,vRightShank,omegaRightShank,IRightShank,RRightShank] = autoGen_RightShank(rOx,rOy,rOz,alpha,beta,gamma,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,rOxdot,rOydot,rOzdot,alphadot,betadot,gammadot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot,q11dot,q12dot,q13dot,q14dot,q15dot,q16dot);
% Right Ankle
[rRightAnkle,vRightAnkle,omegaRightAnkle,IRightAnkle,RRightAnkle] = autoGen_RightAnkle(rOx,rOy,rOz,alpha,beta,gamma,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,rOxdot,rOydot,rOzdot,alphadot,betadot,gammadot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot,q11dot,q12dot,q13dot,q14dot,q15dot,q16dot);
% Right Foot
[rRightFoot,vRightFoot,omegaRightFoot,IRightFoot,RRightFoot] = autoGen_RightFoot(rOx,rOy,rOz,alpha,beta,gamma,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,rOxdot,rOydot,rOzdot,alphadot,betadot,gammadot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot,q11dot,q12dot,q13dot,q14dot,q15dot,q16dot);

% Right Shoulder
[rRightShoulder,vRightShoulder,omegaRightShoulder,IRightShoulder,RRightShoulder] = autoGen_RightShoulder(rOx,rOy,rOz,alpha,beta,gamma,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,rOxdot,rOydot,rOzdot,alphadot,betadot,gammadot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot,q11dot,q12dot,q13dot,q14dot,q15dot,q16dot);
% Right Arm
[rRightArm,vRightArm,omegaRightArm,IRightArm,RRightArm] = autoGen_RightArm(rOx,rOy,rOz,alpha,beta,gamma,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,rOxdot,rOydot,rOzdot,alphadot,betadot,gammadot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot,q11dot,q12dot,q13dot,q14dot,q15dot,q16dot);
% Right Hand
[rRightHand,vRightHand,omegaRightHand,IRightHand,RRightHand] = autoGen_RightHand(rOx,rOy,rOz,alpha,beta,gamma,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,rOxdot,rOydot,rOzdot,alphadot,betadot,gammadot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot,q11dot,q12dot,q13dot,q14dot,q15dot,q16dot);

% Left Hip
[rLeftHip,vLeftHip,omegaLeftHip,ILeftHip,RLeftHip] = autoGen_LeftHip(rOx,rOy,rOz,alpha,beta,gamma,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,rOxdot,rOydot,rOzdot,alphadot,betadot,gammadot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot,q11dot,q12dot,q13dot,q14dot,q15dot,q16dot);
% Left Thigh
[rLeftThigh,vLeftThigh,omegaLeftThigh,ILeftThigh,RLeftThigh] = autoGen_LeftThigh(rOx,rOy,rOz,alpha,beta,gamma,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,rOxdot,rOydot,rOzdot,alphadot,betadot,gammadot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot,q11dot,q12dot,q13dot,q14dot,q15dot,q16dot);
% Left Shank
[rLeftShank,vLeftShank,omegaLeftShank,ILeftShank,RLeftShank] = autoGen_LeftShank(rOx,rOy,rOz,alpha,beta,gamma,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,rOxdot,rOydot,rOzdot,alphadot,betadot,gammadot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot,q11dot,q12dot,q13dot,q14dot,q15dot,q16dot);
% Left Ankle
[rLeftAnkle,vLeftAnkle,omegaLeftAnkle,ILeftAnkle,RLeftAnkle] = autoGen_LeftAnkle(rOx,rOy,rOz,alpha,beta,gamma,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,rOxdot,rOydot,rOzdot,alphadot,betadot,gammadot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot,q11dot,q12dot,q13dot,q14dot,q15dot,q16dot);
% Left Foot
[rLeftFoot,vLeftFoot,omegaLeftFoot,ILeftFoot,RLeftFoot] = autoGen_LeftFoot(rOx,rOy,rOz,alpha,beta,gamma,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,rOxdot,rOydot,rOzdot,alphadot,betadot,gammadot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot,q11dot,q12dot,q13dot,q14dot,q15dot,q16dot);

format short

end

