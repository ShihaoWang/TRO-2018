function Pos = Rigid_Body_Position( State, p )

%% This function is used to get all the velocity of the rigid body

% Position
rOx = State(1);	rOy = State(2);	rOz = State(3);	alpha = State(4);	beta = State(5);	gamma = State(6); 
q1 = State(7);  q2 = State(8);  q3 = State(9);  q4 = State(10);  q5 = State(11);  q6 = State(12);  q7 = State(13);  q8 = State(14);  
q9 = State(15);  q10 = State(16);  q11 = State(17);  q12 = State(18);  q13 = State(19);  q14 = State(20);  q15 = State(21);  q16 = State(22);  

%% rCOM
Pos.rCOM = rCOM_fn(alpha,beta,gamma,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,rOx,rOy,rOz);

%% Position
Pos.rBody = rBody_fn(rOx,rOy,rOz);

Pos.rLeftShoulder = rLeftShoulder_fn(alpha,beta,gamma,rOx,rOy,rOz);
Pos.rLeftArm = rLeftArm_fn(alpha,beta,gamma,q1,q2,rOx,rOy,rOz);
Pos.rLeftHand = rLeftHand_fn(alpha,beta,gamma,q1,q2,q3,rOx,rOy,rOz);

Pos.rRightShoulder = rRightShoulder_fn(alpha,beta,gamma,rOx,rOy,rOz);
Pos.rRightArm = rRightArm_fn(alpha,beta,gamma,q9,q10,rOx,rOy,rOz);
Pos.rRightHand = rRightHand_fn(alpha,beta,gamma,q9,q10,q11,rOx,rOy,rOz);

Pos.rLeftHip = rLeftHip_fn(alpha,beta,gamma,q12,rOx,rOy,rOz);
Pos.rLeftThigh = rLeftThigh_fn(alpha,beta,gamma,q12,q13,rOx,rOy,rOz);
Pos.rLeftShank = rLeftShank_fn(alpha,beta,gamma,q12,q13,q14,rOx,rOy,rOz);
Pos.rLeftAnkle = rLeftAnkle_fn(alpha,beta,gamma,q12,q13,q14,q15,rOx,rOy,rOz);
Pos.rLeftFoot = rLeftFoot_fn(alpha,beta,gamma,q12,q13,q14,q15,q16,rOx,rOy,rOz);

Pos.rRightHip = rRightHip_fn(alpha,beta,gamma,q4,rOx,rOy,rOz);
Pos.rRightThigh = rRightThigh_fn(alpha,beta,gamma,q4,q5,rOx,rOy,rOz);
Pos.rRightShank = rRightShank_fn(alpha,beta,gamma,q4,q5,q6,rOx,rOy,rOz);
Pos.rRightAnkle = rRightAnkle_fn(alpha,beta,gamma,q4,q5,q6,q7,rOx,rOy,rOz);
Pos.rRightFoot = rRightFoot_fn(alpha,beta,gamma,q4,q5,q6,q7,q8,rOx,rOy,rOz);

end

