function Vel = Rigid_Body_Velocity( State, p )

%% This function is used to get all the velocity of the rigid body

% Position
rOx = State(1);	rOy = State(2);	rOz = State(3);	alpha = State(4);	beta = State(5);	gamma = State(6); 
q1 = State(7);  q2 = State(8);  q3 = State(9);  q4 = State(10);  q5 = State(11);  q6 = State(12);  q7 = State(13);  q8 = State(14);  
q9 = State(15);  q10 = State(16);  q11 = State(17);  q12 = State(18);  q13 = State(19);  q14 = State(20);  q15 = State(21);  q16 = State(22);  
% Velocity
rOxdot = State(1+22);	rOydot = State(2+22);	rOzdot = State(3+22);	alphadot = State(4+22);	betadot = State(5+22);	gammadot = State(6+22); 
q1dot = State(7+22);  q2dot = State(8+22);  q3dot = State(9+22);  q4dot = State(10+22);  q5dot = State(11+22);  q6dot = State(12+22);  q7dot = State(13+22);  q8dot = State(14+22);  
q9dot = State(15+22);  q10dot = State(16+22);  q11dot = State(17+22);  q12dot = State(18+22);  q13dot = State(19+22);  q14dot = State(20+22);  q15dot = State(21+22);  q16dot = State(22+22);  

Vel.vBody = vBody_fn(rOxdot,rOydot,rOzdot);

Vel.vLeftShoulder = vLeftShoulder_fn(alpha,alphadot,betadot,beta,gammadot,gamma,rOxdot,rOydot,rOzdot);
Vel.vLeftArm = vLeftArm_fn(alpha,alphadot,betadot,beta,gammadot,gamma,q1,q2,q1dot,q2dot,rOxdot,rOydot,rOzdot);
Vel.vLeftHand = vLeftHand_fn(alpha,alphadot,betadot,beta,gammadot,gamma,q1,q2,q3,q1dot,q2dot,q3dot,rOxdot,rOydot,rOzdot);

Vel.vRightShoulder = vRightShoulder_fn(alpha,alphadot,betadot,beta,gammadot,gamma,rOxdot,rOydot,rOzdot);
Vel.vRightArm = vRightArm_fn(alpha,alphadot,betadot,beta,gammadot,gamma,q9,q10,q10dot,q9dot,rOxdot,rOydot,rOzdot);
Vel.vRightHand = vRightHand_fn(alpha,alphadot,betadot,beta,gammadot,gamma,q9,q10,q11,q10dot,q11dot,q9dot,rOxdot,rOydot,rOzdot);

Vel.vLeftHip = vLeftHip_fn(alpha,alphadot,betadot,beta,gammadot,gamma,q12,q12dot,rOxdot,rOydot,rOzdot);
Vel.vLeftThigh = vLeftThigh_fn(alpha,alphadot,betadot,beta,gammadot,gamma,q12,q13,q12dot,q13dot,rOxdot,rOydot,rOzdot);
Vel.vLeftShank = vLeftShank_fn(alpha,alphadot,betadot,beta,gammadot,gamma,q12,q13,q14,q12dot,q13dot,q14dot,rOxdot,rOydot,rOzdot);
Vel.vLeftAnkle = vLeftAnkle_fn(alpha,alphadot,betadot,beta,gammadot,gamma,q12,q13,q14,q15,q12dot,q13dot,q14dot,q15dot,rOxdot,rOydot,rOzdot);
Vel.vLeftFoot = vLeftFoot_fn(alpha,alphadot,betadot,beta,gammadot,gamma,q12,q13,q14,q15,q16,q12dot,q13dot,q14dot,q15dot,q16dot,rOxdot,rOydot,rOzdot);

Vel.vRightHip = vRightHip_fn(alpha,alphadot,betadot,beta,gammadot,gamma,q4,q4dot,rOxdot,rOydot,rOzdot);
Vel.vRightThigh = vRightThigh_fn(alpha,alphadot,betadot,beta,gammadot,gamma,q4,q5,q4dot,q5dot,rOxdot,rOydot,rOzdot);
Vel.vRightShank = vRightShank_fn(alpha,alphadot,betadot,beta,gammadot,gamma,q4,q5,q6,q4dot,q5dot,q6dot,rOxdot,rOydot,rOzdot);
Vel.vRightAnkle = vRightAnkle_fn(alpha,alphadot,betadot,beta,gammadot,gamma,q4,q5,q6,q7,q4dot,q5dot,q6dot,q7dot,rOxdot,rOydot,rOzdot);
Vel.vRightFoot = vRightFoot_fn(alpha,alphadot,betadot,beta,gammadot,gamma,q4,q5,q6,q7,q8,q4dot,q5dot,q6dot,q7dot,q8dot,rOxdot,rOydot,rOzdot);

end

