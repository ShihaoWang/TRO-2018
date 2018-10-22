function JQRobot_Dynamics()

% This function is used to derive the analytic EoM of the robot
% Here a comparison between the Klampt dynamics engine and my own dynamics
% equation of motion will be compared

% Body coordinates
alpha = [];     beta = [];      gamma= [] ;                         % The meaning 
syms rOx rOy rOz alpha beta gamma real                              % Position and Orientation
syms rOxdot rOydot rOzdot alphadot betadot gammadot real            % Velocity
syms rOxddot rOyddot rOzddot alphaddot betaddot gammaddot real      % Acceleration

syms q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 q11 q12 q13 q14 q15 q16 real
syms q1dot q2dot q3dot q4dot q5dot q6dot q7dot q8dot q9dot q10dot q11dot q12dot q13dot q14dot q15dot q16dot real
syms q1ddot q2ddot q3ddot q4ddot q5ddot q6ddot q7ddot q8ddot q9ddot q10ddot q11ddot q12ddot q13ddot q14ddot q15ddot q16ddot real
% rOx = 0;    rOy = 0;    rOz = 0;    alpha = 0;  beta = 0;   gamma = 0;
% q1 = 0;     q2 = 0;     q3 = 0;     q4 = 0;     q5 = 0;     
% q6 = 0;     q7 = 0;     q8 = 0;     q9 = 0;     q10 = 0;
% q11 = 0;    q12 = 0;    q13 = 0;    q14 = 0;    q15 = 0;    q16 = 0;

g = 9.81;

x = [rOx rOy rOz alpha beta gamma q1 q2 q3 q4 q5 q6 q7 q8 q9 q10 q11 q12 q13 q14 q15 q16]';
xdot = [rOxdot rOydot rOzdot alphadot betadot gammadot q1dot q2dot q3dot q4dot q5dot q6dot q7dot q8dot q9dot q10dot q11dot q12dot q13dot q14dot q15dot q16dot]';
xddot = [rOxddot rOyddot rOzddot alphaddot betaddot gammaddot q1ddot q2ddot q3ddot q4ddot q5ddot q6ddot q7ddot q8ddot q9ddot q10ddot q11ddot q12ddot q13ddot q14ddot q15ddot q16ddot]';

% When we talk about one part, the default point for the analysis is at the center of mass
rBody = [rOx rOy rOz]';
oBody = [alpha beta gamma]';                                        % o* denote the angle of yaw, pitch and roll so it is the aircraft Euler angle
RBody = Euler2Rot(oBody);                                           % R* denote the rotation matrix
IBody = diag([0.0025, 0.0025, 0.001]);                              % I* denote the moment of inertia along the principal axes
mBody = 0.7474;
vBody = jacobian(rBody, x) * xdot;
wBody = jacobian(oBody, x) * xdot;
omegaBody = EulerAngNVel2Omega(oBody, wBody);

% % Now it is the left shoulder
% syms q1 q1dot q1ddot real

rBody2LeftShoulder_self = [-0.06272722, -0.00164721, 0.03188870]';      % This is the relative coordinate between the left shoulder position to the COM of shoulder in the local frame
rLeftShoulder_Ref = rBody + RBody * rBody2LeftShoulder_self;            % This is the reference point of the left shoulder on the robot
oLeftShoulder = [alpha, beta + pi/18, gamma - q1]';
RLeftShoulder = Euler2Rot(oLeftShoulder);
rLeftShoulder_Ref2COM = [ -0.00423081 0 0]';
rLeftShoulder = rLeftShoulder_Ref + RLeftShoulder * rLeftShoulder_Ref2COM;
ILeftShoulder = diag([3.6 * 10^(-6), 7.698 * 10^(-7), 3.6 * 10^(-6)]);
mLeftShoulder = 0.0087;
vLeftShoulder = jacobian(rLeftShoulder, x) * xdot;
wLeftShoulder = jacobian(oLeftShoulder, x) * xdot; 
omegaLeftShoulder = EulerAngNVel2Omega(oLeftShoulder, wLeftShoulder);

% Now it is the left arm
rLeftShoulder2LeftArm_self = [-0.01950677, 0.02575, 0]';        % One easy trick to allow for the comparison 
rLeftArm_Ref = rLeftShoulder_Ref + RLeftShoulder * rLeftShoulder2LeftArm_self;
oLeftArm = [alpha, beta + pi/18 + q2, gamma - q1]';
RLeftArm = Euler2Rot(oLeftArm);
rLeftArm_Ref2COM = [-0.03595415, -0.02621678, 0]';
rLeftArm = rLeftArm_Ref + RLeftArm * rLeftArm_Ref2COM;
ILeftArm = diag([3.515445 * 10^(-5), 1.25 * 10^(-4), 1.25 * 10^(-4)]);
mLeftArm = 0.135;
vLeftArm = jacobian(rLeftArm, x) * xdot;
wLeftArm = jacobian(oLeftArm, x) * xdot;
omegaLeftArm = EulerAngNVel2Omega(oLeftArm, wLeftArm);

% Now it is the left hand
rLeftArm2LeftHand_self = [-0.072 0 0 ]';
rLeftHand_Ref = rLeftArm_Ref + RLeftArm * rLeftArm2LeftHand_self;
oLeftHand = [alpha, beta + pi/18 + q2 + q3, gamma - q1]';
RLeftHand = Euler2Rot(oLeftHand);
rLeftHand_Ref2COM = [-0.04614073, -0.02968938, 0]';
rLeftHand = rLeftHand_Ref + RLeftHand * rLeftHand_Ref2COM;
ILeftHand = diag([9 * 10^(-6), 4 * 10^(-5), 4 * 10^(-5)])';
mLeftHand = 0.0315;
vLeftHand = jacobian(rLeftHand, x) * xdot;
wLeftHand = jacobian(oLeftHand, x) * xdot;
omegaLeftHand = EulerAngNVel2Omega(oLeftHand, wLeftHand);

% Now it is the right shoulder
rBody2RightShoulder_self = [0.06272722, -0.00164721, 0.03188870]';   % This is the relative coordinate between the left shoulder position to the COM of shoulder in the local frame
rRightShoulder_Ref = rBody + RBody * rBody2RightShoulder_self;
oRightShoulder = [alpha, beta - pi/18, gamma + q9]';
RRightShoulder = Euler2Rot(oRightShoulder);
rRightShoulder_Ref2COM = [ 0.00423081 0 0]';
rRightShoulder = rRightShoulder_Ref + RRightShoulder * rRightShoulder_Ref2COM;
IRightShoulder = diag([3.6 * 10^(-6), 7.698 * 10^(-7), 3.6 * 10^(-6)]);
mRightShoulder = 0.0087;
vRightShoulder = jacobian(rRightShoulder, x) * xdot;
wRightShoulder = jacobian(oRightShoulder, x) * xdot;
omegaRightShoulder = EulerAngNVel2Omega(oRightShoulder, wRightShoulder);

% Now it is the right arm
rRightShoulder2RightArm_self = [0.01950677, 0.02575, 0]';
rRightArm_Ref = rRightShoulder_Ref + RRightShoulder * rRightShoulder2RightArm_self;
oRightArm = [alpha, beta - pi/18 + q10, gamma + q9]';
RRightArm = Euler2Rot(oRightArm);
rRightArm_Ref2COM = [0.03595415, -0.02621678, 0]';
rRightArm = rRightArm_Ref + RRightArm * rRightArm_Ref2COM;
IRightArm = diag([3.515445 * 10^(-5), 1.25 * 10^(-4), 1.25 * 10^(-4)]);
mRightArm = 0.135;
vRightArm = jacobian(rRightArm, x) * xdot;
wRightArm = jacobian(oRightArm, x) * xdot;
omegaRightArm = EulerAngNVel2Omega(oRightArm, wRightArm);

% Now it is the right hand
rRightArm2RightHand_self = [0.072 0 0 ]';
rRightHand_Ref = rRightArm_Ref + RRightArm * rRightArm2RightHand_self;
oRightHand = [alpha, beta - pi/18 + q10 + q11, gamma + q9]';
RRightHand = Euler2Rot(oRightHand);
rRightHand_Ref2COM = [0.04614073, -0.02968937, 0]';
rRightHand = rRightHand_Ref + RRightHand * rRightHand_Ref2COM;
IRightHand = diag([9 * 10^(-6), 4 * 10^(-5), 4 * 10^(-5)])';
mRightHand = 0.0315;
vRightHand = jacobian(rRightHand, x) * xdot;
wRightHand = jacobian(oRightHand, x) * xdot;
omegaRightHand = EulerAngNVel2Omega(oRightHand, wRightHand);

% Now it is the left hip
rBody2LeftHip_Self = [-0.03172834, 0.02330279, -0.09266659]';
rLeftHip_Ref = rBody + RBody * rBody2LeftHip_Self;
oLeftHip = [alpha, beta - q12, gamma]';
RLeftHip = Euler2Rot(oLeftHip);
rLeftHip_Ref2COM = [-0.01065566, -0.01458538, -0.00268635]';
rLeftHip = rLeftHip_Ref + RLeftHip * rLeftHip_Ref2COM;
ILeftHip = diag([5 * 10^(-5), 5 * 10^(-5), 5 * 10^(-5)])';
mLeftHip = 0.0832;
vLeftHip = jacobian(rLeftHip, x) * xdot;
wLeftHip = jacobian(oLeftHip, x) * xdot;
omegaLeftHip = EulerAngNVel2Omega(oLeftHip, wLeftHip);

% Now it is the left thigh
rLeftHip2LeftThigh_Self = [0.0165, -0.03225, -0.035]';
rLeftThigh_Ref = rLeftHip_Ref + RLeftHip * rLeftHip2LeftThigh_Self;
oLeftThigh = [alpha, beta - q12, gamma - q13];
RLeftThigh = Euler2Rot(oLeftThigh);
rLeftThigh_Ref2COM = [-0.0262606, 0, -0.01304877]';
rLeftThigh = rLeftThigh_Ref + RLeftThigh * rLeftThigh_Ref2COM;
ILeftThigh = diag([1.8 * 10^(-5), 3 * 10^(-4), 1.8 * 10^(-5)]);
mLeftThigh = 0.077;
vLeftThigh = jacobian(rLeftThigh, x) * xdot;
wLeftThigh = jacobian(oLeftThigh, x) * xdot;
omegaLeftThigh = EulerAngNVel2Omega(oLeftThigh, wLeftThigh);

% Now it is the left shank
rLeftThigh2LeftShank_Self = [-0.001, 0, -0.0555]';
rLeftShank_Ref = rLeftThigh_Ref + RLeftThigh * rLeftThigh2LeftShank_Self;
oLeftShank = [alpha, beta - q12, gamma - q13 - q14]';
RLeftShank = Euler2Rot(oLeftShank);
rLeftShank_Ref2COM = [-0.02638657, 0, -0.0329934]';
rLeftShank = rLeftShank_Ref + RLeftShank * rLeftShank_Ref2COM;
ILeftShank = diag([1 * 10^(-4), 1 * 10^(-4), 3.7987 * 10^(-5)]);
mLeftShank = 0.1381;
vLeftShank = jacobian(rLeftShank, x) * xdot;
wLeftShank = jacobian(oLeftShank, x) * xdot;
omegaLeftShank = EulerAngNVel2Omega(oLeftShank, wLeftShank);

% Now it is the left ankle
rLeftShank2LeftAnkle_Self = [0, 0, -0.066]';
rLeftAnkle_Ref = rLeftShank_Ref + RLeftShank * rLeftShank2LeftAnkle_Self;
oLeftAnkle = [alpha, beta - q12, gamma - q13 - q14 - q15]';
RLeftAnkle = Euler2Rot(oLeftAnkle);
rLeftAnkle_Ref2COM = [-0.02590533, 0.00688525, -0.0340719]';
rLeftAnkle = rLeftAnkle_Ref + RLeftAnkle * rLeftAnkle_Ref2COM;
ILeftAnkle = diag([3 * 10^(-5), 3 * 10^(-5), 3 * 10^(-5)]);
mLeftAnkle = 0.0892;
vLeftAnkle = jacobian(rLeftAnkle, x) * xdot;
wLeftAnkle = jacobian(oLeftAnkle, x) * xdot;
omegaLeftAnkle = EulerAngNVel2Omega(oLeftAnkle, wLeftAnkle);

% Now it is the left foot
rLeftAnkle2LeftFoot_Self = [-0.0165, 0.03325, -0.035]';
rLeftFoot_Ref = rLeftAnkle_Ref + RLeftAnkle * rLeftAnkle2LeftFoot_Self;
oLeftFoot = [alpha, beta - q12 + q16, gamma - q13 - q14 - q15]';
RLeftFoot = Euler2Rot(oLeftFoot);
rLeftFoot_Ref2COM = [-0.020799, -0.02575090, -0.02343266]';
rLeftFoot = rLeftFoot_Ref + RLeftFoot * rLeftFoot_Ref2COM;
ILeftFoot = diag([1 * 10^(-4), 5 * 10^(-5), 1.5 * 10^(-4)]);
mLeftFoot = 0.0636;
vLeftFoot = jacobian(rLeftFoot, x) * xdot;
wLeftFoot = jacobian(oLeftFoot, x) * xdot;
omegaLeftFoot = EulerAngNVel2Omega(oLeftFoot, wLeftFoot);

% Now it is the right hip
rBody2RightHip_Self = [0.03172834, 0.02330279, -0.09266659]';
rRightHip_Ref = rBody + RBody * rBody2RightHip_Self;
oRightHip = [alpha, beta - q4, gamma];
RRightHip = Euler2Rot(oRightHip);
rRightHip_Ref2COM = [0.01065566, -0.01458538, -0.00268635]'; 
rRightHip = rRightHip_Ref + RRightHip * rRightHip_Ref2COM;
IRightHip = diag([0.00005, 0.00005, 0.00005])';
mRightHip = 0.0832;
vRightHip = jacobian(rRightHip, x) * xdot;
wRightHip = jacobian(oRightHip, x) * xdot;
omegaRightHip = EulerAngNVel2Omega(oRightHip, wRightHip);

% Now it is the right thigh
rRightHip2RightThigh_Self = [-0.0165, -0.03225, -0.035]';
rRightThigh_Ref = rRightHip_Ref + RRightHip * rRightHip2RightThigh_Self;
oRightThigh = [alpha, beta - q4, gamma + q5];
RRightThigh = Euler2Rot(oRightThigh);
rRightThigh_Ref2COM = [0.0262606, 0, -0.01304877]';
rRightThigh = rRightThigh_Ref + RRightThigh * rRightThigh_Ref2COM;
IRightThigh = diag([1.8 * 10^(-5), 3 * 10^(-4), 1.8 * 10^(-5)]);
mRightThigh = 0.077;
vRightThigh = jacobian(rRightThigh, x) * xdot;
wRightThigh = jacobian(oRightThigh, x) * xdot;
omegaRightThigh = EulerAngNVel2Omega(oRightThigh, wRightThigh);

% Now it is the right shank
rRightThigh2RightShank_Self = [0.001, 0, -0.0555]';
rRightShank_Ref = rRightThigh_Ref + RRightThigh * rRightThigh2RightShank_Self;
oRightShank = [alpha, beta - q4, gamma + q5 + q6]';
RRightShank = Euler2Rot(oRightShank);
rRightShank_Ref2COM = [0.02638657, 0, -0.0329934]';
rRightShank = rRightShank_Ref + RRightShank * rRightShank_Ref2COM;
IRightShank = diag([1*10^(-4), 1*10^(-4), 3.7987 * 10^(-5)]);
mRightShank = 0.1381;
vRightShank = jacobian(rRightShank, x) * xdot;
wRightShank = jacobian(oRightShank, x) * xdot;
omegaRightShank = EulerAngNVel2Omega(oRightShank, wRightShank);

% Now it is the right ankle
rRightShank2RightAnkle_Self = [0, 0, -0.066]';
rRightAnkle_Ref = rRightShank_Ref + RRightShank * rRightShank2RightAnkle_Self;
oRightAnkle = [alpha, beta - q4, gamma + q5 + q6 - q7]';
RRightAnkle = Euler2Rot(oRightAnkle);
rRightAnkle_Ref2COM = [0.02590533, 0.00688525, -0.0340719]';
rRightAnkle = rRightAnkle_Ref + RRightAnkle * rRightAnkle_Ref2COM;
IRightAnkle = diag([3 * 10^(-5), 3 * 10^(-5), 3 * 10^(-5)]);
mRightAnkle = 0.0892;
vRightAnkle = jacobian(rRightAnkle, x) * xdot;
wRightAnkle = jacobian(oRightAnkle, x) * xdot;
omegaRightAnkle = EulerAngNVel2Omega(oRightAnkle, wRightAnkle);

% Now it is the right foot
rRightAnkle2RightFoot_Self = [0.0165, 0.03325, -0.035]';
rRightFoot_Ref = rRightAnkle_Ref + RRightAnkle * rRightAnkle2RightFoot_Self;
oRightFoot = [alpha, beta - q4 + q8, gamma + q5 + q6 - q7]';
RRightFoot = Euler2Rot(oRightFoot);
rRightFoot_Ref2COM = [0.020799, -0.02575090, -0.02343266]';
rRightFoot = rRightFoot_Ref + RRightFoot * rRightFoot_Ref2COM;
IRightFoot = diag([1 * 10^(-4), 5 * 10^(-5), 1.5 * 10^(-4)]);
mRightFoot = 0.0636;
vRightFoot = jacobian(rRightFoot, x) * xdot;
wRightFoot = jacobian(oRightFoot, x) * xdot;
omegaRightFoot = EulerAngNVel2Omega(oRightFoot, wRightFoot);

%% Given value to structure
p.mBody = mBody;

p.mLeftShoulder = mLeftShoulder;            p.mLeftArm = mLeftArm;              p.mLeftHand = mLeftHand;

p.mRightShoulder = mRightShoulder;          p.mRightArm = mRightArm;            p.mRightHand = mRightHand;

p.mLeftHip = mLeftHip;                      p.mLeftThigh = mLeftThigh;          p.mLeftShank = mLeftShank;          p.mLeftAnkle = mLeftAnkle;
p.mLeftFoot = mLeftFoot;

p.mRightHip = mRightHip;                    p.mRightThigh = mRightThigh;        p.mRightShank = mRightShank;        p.mRightAnkle = mRightAnkle;
p.mRightFoot = mRightFoot;

%% Individual moment of inertia
p.IBody = IBody;

p.ILeftShoulder = ILeftShoulder;            p.ILeftArm = ILeftArm;              p.ILeftHand = ILeftHand;

p.IRightShoulder = IRightShoulder;          p.IRightArm = IRightArm;            p.IRightHand = IRightHand;

p.ILeftHip = ILeftHip;                      p.ILeftThigh = ILeftThigh;          p.ILeftShank = ILeftShank;          p.ILeftAnkle = ILeftAnkle;
p.ILeftFoot = ILeftFoot;

p.IRightHip = IRightHip;                    p.IRightThigh = IRightThigh;        p.IRightShank = IRightShank;        p.IRightAnkle = IRightAnkle;
p.IRightFoot = IRightFoot;

save('p.mat','p');

% %% Here the robot mass and inertia have been formulated. 
% % Then the next job is to write a function such that given an arbitrary robot state, this function will output the robot's average angular velocity.
% % Position
% matlabFunction(rBody,'file','rBody_fn.m');
% 
% matlabFunction(rLeftShoulder, 'file','rLeftShoulder_fn.m');
% matlabFunction(rLeftArm, 'file','rLeftArm_fn.m');
% matlabFunction(rLeftHand, 'file','rLeftHand_fn.m');
% matlabFunction(rRightShoulder, 'file','rRightShoulder_fn.m');
% matlabFunction(rRightArm, 'file','rRightArm_fn.m');
% matlabFunction(rRightHand, 'file','rRightHand_fn.m');
% 
% matlabFunction(rLeftHip, 'file','rLeftHip_fn.m');
% matlabFunction(rLeftThigh, 'file','rLeftThigh_fn.m');
% matlabFunction(rLeftShank, 'file','rLeftShank_fn.m');
% matlabFunction(rLeftAnkle, 'file','rLeftAnkle_fn.m');
% matlabFunction(rLeftFoot, 'file','rLeftFoot_fn.m');
% 
% matlabFunction(rRightHip, 'file','rRightHip_fn.m');
% matlabFunction(rRightThigh, 'file','rRightThigh_fn.m');
% matlabFunction(rRightShank, 'file','rRightShank_fn.m');
% matlabFunction(rRightAnkle, 'file','rRightAnkle_fn.m');
% matlabFunction(rRightFoot, 'file','rRightFoot_fn.m');
% 
% % Velocity
% matlabFunction(vBody,'file','vBody_fn.m');
% 
% matlabFunction(vLeftShoulder, 'file','vLeftShoulder_fn.m');
% matlabFunction(vLeftArm, 'file','vLeftArm_fn.m');
% matlabFunction(vLeftHand, 'file','vLeftHand_fn.m');
% matlabFunction(vRightShoulder, 'file','vRightShoulder_fn.m');
% matlabFunction(vRightArm, 'file','vRightArm_fn.m');
% matlabFunction(vRightHand, 'file','vRightHand_fn.m');
% 
% matlabFunction(vLeftHip, 'file','vLeftHip_fn.m');
% matlabFunction(vLeftThigh, 'file','vLeftThigh_fn.m');
% matlabFunction(vLeftShank, 'file','vLeftShank_fn.m');
% matlabFunction(vLeftAnkle, 'file','vLeftAnkle_fn.m');
% matlabFunction(vLeftFoot, 'file','vLeftFoot_fn.m');
% 
% matlabFunction(vRightHip, 'file','vRightHip_fn.m');
% matlabFunction(vRightThigh, 'file','vRightThigh_fn.m');
% matlabFunction(vRightShank, 'file','vRightShank_fn.m');
% matlabFunction(vRightAnkle, 'file','vRightAnkle_fn.m');
% matlabFunction(vRightFoot, 'file','vRightFoot_fn.m');
% 
% %% Rotation matrix
% matlabFunction(RBody,'file','RotBody_fn.m');
% 
% matlabFunction(RLeftShoulder, 'file','RotLeftShoulder_fn.m');
% matlabFunction(RLeftArm, 'file','RotLeftArm_fn.m');
% matlabFunction(RLeftHand, 'file','RotLeftHand_fn.m');
% matlabFunction(RRightShoulder, 'file','RotRightShoulder_fn.m');
% matlabFunction(RRightArm, 'file','RotRightArm_fn.m');
% matlabFunction(RRightHand, 'file','RotRightHand_fn.m');
% 
% matlabFunction(RLeftHip, 'file','RotLeftHip_fn.m');
% matlabFunction(RLeftThigh, 'file','RotLeftThigh_fn.m');
% matlabFunction(RLeftShank, 'file','RotLeftShank_fn.m');
% matlabFunction(RLeftAnkle, 'file','RotLeftAnkle_fn.m');
% matlabFunction(RLeftFoot, 'file','RotLeftFoot_fn.m');
% 
% matlabFunction(RRightHip, 'file','RotRightHip_fn.m');
% matlabFunction(RRightThigh, 'file','RotRightThigh_fn.m');
% matlabFunction(RRightShank, 'file','RotRightShank_fn.m');
% matlabFunction(RRightAnkle, 'file','RotRightAnkle_fn.m');
% matlabFunction(RRightFoot, 'file','RotRightFoot_fn.m');

% %% Angular velocity
% 
% matlabFunction(omegaBody,'file','omegaBody_fn.m');
% 
% matlabFunction(omegaLeftShoulder, 'file','omegaLeftShoulder_fn.m');
% matlabFunction(omegaLeftArm, 'file','omegaLeftArm_fn.m');
% matlabFunction(omegaLeftHand, 'file','omegaLeftHand_fn.m');
% 
% matlabFunction(omegaRightShoulder, 'file','omegaRightShoulder_fn.m');
% matlabFunction(omegaRightArm, 'file','omegaRightArm_fn.m');
% matlabFunction(omegaRightHand, 'file','omegaRightHand_fn.m');
% 
% matlabFunction(omegaLeftHip, 'file','omegaLeftHip_fn.m');
% matlabFunction(omegaLeftThigh, 'file','omegaLeftThigh_fn.m');
% matlabFunction(omegaLeftShank, 'file','omegaLeftShank_fn.m');
% matlabFunction(omegaLeftAnkle, 'file','omegaLeftAnkle_fn.m');
% matlabFunction(omegaLeftFoot, 'file','omegaLeftFoot_fn.m');
% 
% matlabFunction(omegaRightHip, 'file','omegaRightHip_fn.m');
% matlabFunction(omegaRightThigh, 'file','omegaRightThigh_fn.m');
% matlabFunction(omegaRightShank, 'file','omegaRightShank_fn.m');
% matlabFunction(omegaRightAnkle, 'file','omegaRightAnkle_fn.m');
% matlabFunction(omegaRightFoot, 'file','omegaRightFoot_fn.m');

% The next job is to evaluate the ceter of mass and inertia of given configuration 

% The relative moment of inertia measured with respect to the center of mass is measured according to distance

% %% The expression of the center of mass
% rCOM_tot = mBody * rBody + mLeftShoulder * rLeftShoulder + mLeftArm * rLeftArm + mLeftHand * rLeftHand + ...
%                        mRightShoulder * rRightShoulder + mRightArm * rRightArm + mRightHand * rRightHand + ...
%                        mLeftHip * rLeftHip + mLeftThigh * rLeftThigh + mLeftShank * rLeftShank + mLeftAnkle * rLeftAnkle + mLeftFoot * rLeftFoot + ...
%                        mRightHip * rRightHip + mRightThigh * rRightThigh + mRightShank * rRightShank + mRightAnkle * rRightAnkle + mRightFoot * rRightFoot;
% m_tot = mBody + mLeftShoulder + mLeftArm + mLeftHand + mRightShoulder + mRightArm + mRightHand + ...
%                 mLeftHip + mLeftThigh + mLeftShank + mLeftAnkle + mLeftFoot + ...
%                 mRightHip + mRightThigh + mRightShank + mRightAnkle + mRightFoot;
% 
% rCOM = simplify(rCOM_tot/m_tot);
% 
% matlabFunction(rCOM, 'file','rCOM_fn.m');
% 
% %% The next expression is the linear momentum of the robot at COM
% 
% L_COM = mBody * vBody + mLeftShoulder * vLeftShoulder + mLeftArm * vLeftArm + mLeftHand * vLeftHand + ...
%                         mRightShoulder * vRightShoulder + mRightArm * vRightArm + mRightHand * vRightHand + ...
%                         mLeftHip * vLeftHip + mLeftThigh * vLeftThigh + mLeftShank * vLeftShank + mLeftAnkle * vLeftAnkle + mLeftFoot * vLeftFoot + ...
%                         mRightHip * vRightHip + mRightThigh * vRightThigh + mRightShank * vRightShank + mRightAnkle * vRightAnkle + mRightFoot * vRightFoot;
% L_COM = simplify(L_COM);
% matlabFunction(L_COM, 'file','L_COM_fn.m');

%% The next expression is the angular momentum of the robot at instantaneous COM
% H_COM = cross(rBody - rCOM, mBody * vBody) + cross(rLeftShoulder - rCOM, mLeftShoulder * vLeftShoulder) + cross(rLeftArm - rCOM, mLeftArm * vLeftArm) + ...
%         cross(rLeftHand - rCOM, mLeftHand * vLeftHand) + cross(rRightShoulder - rCOM, mRightShoulder * vRightShoulder) + cross(rRightArm - rCOM, mRightArm * vRightArm) + ...
%         cross(rRightHand - rCOM, mRightHand * vRightHand) + cross(rLeftHip - rCOM, mLeftHip * vLeftHip) + cross(rLeftThigh - rCOM, mLeftThigh * vLeftThigh) + ...
%         cross(rLeftShank - rCOM, mLeftShank * vLeftShank) + cross(rLeftAnkle - rCOM, mLeftAnkle * vLeftAnkle) + cross(rLeftFoot - rCOM, mLeftFoot * vLeftFoot) + ...
%         cross(rRightHip - rCOM, mRightHip * vRightHip) + cross(rRightThigh - rCOM, mRightThigh * vRightThigh) + cross(rRightShank - rCOM, mRightShank * vRightShank) + cross(rRightAnkle - rCOM, mRightAnkle * vRightAnkle) + cross(rRightFoot - rCOM, mRightFoot * vRightFoot);

% H_COM = simplify(H_COM);
% matlabFunction(H_COM, 'file','H_COM_fn.m');

%% Equation of Motion

%% Kinetic energy
%% Body
Ek_Body = 1/2 * mBody * dot(vBody, vBody) + 1/2 * omegaBody' * IBody * wBody;
Ek_Body = simplify(Ek_Body);

%% Left upper limb
Ek_LeftShoulder = 1/2 * mLeftShoulder * dot(vLeftShoulder, vLeftShoulder) + 1/2 * omegaLeftShoulder' * ILeftShoulder * omegaLeftShoulder;
Ek_LeftArm = 1/2 * mLeftArm * dot(vLeftArm, vLeftArm) + 1/2 * omegaLeftArm' * ILeftArm * omegaLeftArm;
Ek_LeftHand = 1/2 * mLeftHand * dot(vLeftHand, vLeftHand) + 1/2 * omegaLeftHand' * ILeftHand * omegaLeftHand;

Ek_LeftShoulder = simplify(Ek_LeftShoulder);
Ek_LeftArm = simplify(Ek_LeftArm);
Ek_LeftHand = simplify(Ek_LeftHand); 

%% Right upper limb
Ek_RightShoulder = 1/2 * mRightShoulder * dot(vRightShoulder, vRightShoulder) + 1/2 * omegaRightShoulder' * IRightShoulder * omegaRightShoulder;
Ek_RightArm = 1/2 * mRightArm * dot(vRightArm, vRightArm) + 1/2 * omegaRightArm' * IRightArm * omegaRightArm;
Ek_RightHand = 1/2 * mRightHand * dot(vRightHand, vRightHand) + 1/2 * omegaRightHand' * IRightHand * omegaRightHand;

Ek_RightShoulder = simplify(Ek_RightShoulder);
Ek_RightArm = simplify(Ek_RightArm);
Ek_RightHand = simplify(Ek_RightHand);

%% Left lower limb
Ek_LeftHip = 1/2 * mLeftHip * dot(vLeftHip, vLeftHip) + 1/2 * omegaLeftHip' * ILeftHip * omegaLeftHip;
Ek_LeftThigh = 1/2 * mLeftThigh * dot(vLeftThigh, vLeftThigh) + 1/2 * omegaLeftThigh' * ILeftThigh * omegaLeftThigh;
Ek_LeftShank = 1/2 * mLeftShank * dot(vLeftShank, vLeftShank) + 1/2 * omegaLeftShank' * ILeftShank * omegaLeftShank;
Ek_LeftAnkle = 1/2 * mLeftAnkle * dot(vLeftAnkle, vLeftAnkle) + 1/2 * omegaLeftAnkle' * ILeftAnkle * omegaLeftAnkle;
Ek_LeftFoot = 1/2 * mLeftFoot * dot(vLeftFoot, vLeftFoot) + 1/2 * omegaLeftFoot' * ILeftFoot * omegaLeftFoot;

Ek_LeftHip = simplify(Ek_LeftHip);
Ek_LeftThigh = simplify(Ek_LeftThigh);
Ek_LeftShank = simplify(Ek_LeftShank);
Ek_LeftAnkle = simplify(Ek_LeftAnkle);
Ek_LeftFoot = simplify(Ek_LeftFoot);

%% Right lower limb
Ek_RightHip = 1/2 * mRightHip * dot(vRightHip, vRightHip) + 1/2 * omegaRightHip' * IRightHip * omegaRightHip;
Ek_RightThigh = 1/2 * mRightThigh * dot(vRightThigh, vRightThigh) + 1/2 * omegaRightThigh' * IRightThigh * omegaRightThigh;
Ek_RightShank = 1/2 * mRightShank * dot(vRightShank, vRightShank) + 1/2 * omegaRightShank' * IRightShank * omegaRightShank;
Ek_RightAnkle = 1/2 * mRightAnkle * dot(vRightAnkle, vRightAnkle) + 1/2 * omegaRightAnkle' * IRightAnkle * omegaRightAnkle;
Ek_RightFoot = 1/2 * mRightFoot * dot(vRightFoot, vRightFoot) + 1/2 * omegaRightFoot' * IRightFoot * omegaRightFoot;

Ek_RightHip = simplify(Ek_RightHip);
Ek_RightThigh = simplify(Ek_RightThigh);
Ek_RightShank = simplify(Ek_RightShank);
Ek_RightAnkle = simplify(Ek_RightAnkle);
Ek_RightFoot = simplify(Ek_RightFoot);

Ek = Ek_Body + Ek_LeftShoulder + Ek_LeftArm + Ek_LeftHand + Ek_RightShoulder + Ek_RightArm + Ek_RightHand + ...
     Ek_LeftHip + Ek_LeftThigh + Ek_LeftShank + Ek_LeftAnkle + Ek_LeftFoot + ...
     Ek_RightHip + Ek_RightThigh + Ek_RightShank + Ek_RightAnkle + Ek_RightFoot;
Ek = simplify(Ek);
 
%% Potential energy
Ep_Body = mBody * g * rBody(3);

Ep_LeftShoulder = mLeftShoulder * g * rLeftShoulder(3);
Ep_LeftArm = mLeftArm * g * rLeftArm(3);
Ep_LeftHand = mLeftHand * g * rLeftHand(3);

Ep_RightShoulder = mRightShoulder * g * rRightShoulder(3);
Ep_RightArm = mRightArm * g * rRightArm(3);
Ep_RightHand = mRightHand * g * rRightHand(3);

Ep_LeftHip = mLeftHip * g * rLeftHip(3);
Ep_LeftThigh = mLeftThigh * g * rLeftThigh(3);
Ep_LeftShank = mLeftShank * g * rLeftShank(3);
Ep_LeftAnkle = mLeftAnkle * g * rLeftAnkle(3);
Ep_LeftFoot = mLeftFoot * g * rLeftFoot(3);

Ep_RightHip = mRightHip * g * rRightHip(3);
Ep_RightThigh = mRightThigh * g * rRightThigh(3);
Ep_RightShank = mRightShank * g * rRightShank(3);
Ep_RightAnkle = mRightAnkle * g * rRightAnkle(3);
Ep_RightFoot = mRightFoot * g * rRightFoot(3);

Ep = Ep_Body + Ep_LeftShoulder + Ep_LeftArm + Ep_LeftHand + Ep_RightShoulder + Ep_RightArm + Ep_RightHand + ...
     Ep_LeftHip + Ep_LeftThigh + Ep_LeftShank + Ep_LeftAnkle + Ep_LeftFoot + ...
     Ep_RightHip + Ep_RightThigh + Ep_RightShank + Ep_RightAnkle + Ep_RightFoot;
Ep = simplify(Ep); 
L = Ek - Ep;

% % Body 2 Left Shoulder
% Line_Plot(rBody, rLeftShoulder)
% hold on
% Line_Plot(rLeftShoulder, rLeftArm)
% hold on
% Line_Plot(rLeftArm, rLeftHand)
% hold on
% Line_Plot(rBody, rRightShoulder)
% hold on
% 
% Line_Plot(rRightShoulder, rRightArm)
% hold on
% 
% Line_Plot(rRightArm, rRightHand)
% hold on
% grid on
% xlabel('x axis')
% ylabel('y axis')
% zlabel('z axis')

end

function Line_Plot(rPos1, rPos2)

plot3([rPos1(1),rPos2(1)], [rPos1(2),rPos2(2)], [rPos1(3),rPos2(3)], 'LineWidth',2)

end

function Rot = Euler2Rot(angle_array)

% First Yaw, then Pitch and last Roll

Rot = Rot_Mat(angle_array(1), 'z') * Rot_Mat(angle_array(2), 'y') * Rot_Mat(angle_array(3), 'x');

end

function Rot = Rot_Mat(theta, type)

% This is using the intrisinc manner to express the rotation matrix

% The idea of this matrix is to project the vector from the body frame into
% the global frame

if type == 'z'
    % This is the yaw (z axis)
    Rot = [ cos(theta) -sin(theta) 0;...
            sin(theta) cos(theta) 0;...
            0 0 1];
else
    if type == 'y'
        % This is the pitch (y axis)
        Rot = [ cos(theta) 0 sin(theta);...
                0 1 0;
                -sin(theta) 0 cos(theta)];
    else
        % This is the roll (x axis)
         Rot = [ 1 0 0;...
                 0 cos(theta) -sin(theta);...
                 0 sin(theta) cos(theta)];
    end
end
end

