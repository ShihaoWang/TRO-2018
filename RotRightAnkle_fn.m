function RRightAnkle = RotRightAnkle_fn(alpha,beta,gamma,q4,q5,q6,q7)
%ROTRIGHTANKLE_FN
%    RRIGHTANKLE = ROTRIGHTANKLE_FN(ALPHA,BETA,GAMMA,Q4,Q5,Q6,Q7)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    18-Oct-2018 14:10:24

t2 = beta-q4;
t3 = cos(alpha);
t4 = gamma+q5+q6-q7;
t5 = sin(alpha);
t6 = sin(t4);
t7 = sin(t2);
t8 = cos(t4);
t9 = cos(t2);
RRightAnkle = reshape([t3.*t9,t5.*t9,-t7,-t5.*t8+t3.*t6.*t7,t3.*t8+t5.*t6.*t7,t6.*t9,t5.*t6+t3.*t7.*t8,-t3.*t6+t5.*t7.*t8,t8.*t9],[3,3]);