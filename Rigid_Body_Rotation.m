function Rot = Rigid_Body_Rotation( State, p )

%% This function is used to get all the velocity of the rigid body

% Position
rOx = State(1);	rOy = State(2);	rOz = State(3);	alpha = State(4);	beta = State(5);	gamma = State(6); 
q1 = State(7);  q2 = State(8);  q3 = State(9);  q4 = State(10);  q5 = State(11);  q6 = State(12);  q7 = State(13);  q8 = State(14);  
q9 = State(15);  q10 = State(16);  q11 = State(17);  q12 = State(18);  q13 = State(19);  q14 = State(20);  q15 = State(21);  q16 = State(22);  

%% Rotation matrix

Rot.RotBody = RotBody_fn(alpha,beta,gamma);

Rot.RotLeftShoulder = RotLeftShoulder_fn(alpha,beta,gamma,q1);
Rot.RotLeftArm = RotLeftArm_fn(alpha,beta,gamma,q1,q2);
Rot.RotLeftHand = RotLeftHand_fn(alpha,beta,gamma,q1,q2,q3);

Rot.RotRightShoulder = RotRightShoulder_fn(alpha,beta,gamma,q9);
Rot.RotRightArm = RotRightArm_fn(alpha,beta,gamma,q9,q10);
Rot.RotRightHand = RotRightHand_fn(alpha,beta,gamma,q9,q10,q11);

Rot.RotLeftHip = RotLeftHip_fn(alpha,beta,gamma,q12);
Rot.RotLeftThigh = RotLeftThigh_fn(alpha,beta,gamma,q12,q13);
Rot.RotLeftShank = RotLeftShank_fn(alpha,beta,gamma,q12,q13,q14);
Rot.RotLeftAnkle = RotLeftAnkle_fn(alpha,beta,gamma,q12,q13,q14,q15);
Rot.RotLeftFoot = RotLeftFoot_fn(alpha,beta,gamma,q12,q13,q14,q15,q16);

Rot.RotRightHip = RotRightHip_fn(alpha,beta,gamma,q4);
Rot.RotRightThigh = RotRightThigh_fn(alpha,beta,gamma,q4,q5);
Rot.RotRightShank = RotRightShank_fn(alpha,beta,gamma,q4,q5,q6);
Rot.RotRightAnkle = RotRightAnkle_fn(alpha,beta,gamma,q4,q5,q6,q7);
Rot.RotRightFoot = RotRightFoot_fn(alpha,beta,gamma,q4,q5,q6,q7,q8);

end

