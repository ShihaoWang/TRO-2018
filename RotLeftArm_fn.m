function RLeftArm = RotLeftArm_fn(alpha,beta,gamma,q1,q2)
%ROTLEFTARM_FN
%    RLEFTARM = ROTLEFTARM_FN(ALPHA,BETA,GAMMA,Q1,Q2)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    18-Oct-2018 14:10:22

t2 = gamma-q1;
t3 = pi.*(1.0./1.8e1);
t4 = beta+q2+t3;
t5 = cos(alpha);
t6 = sin(t2);
t7 = sin(alpha);
t8 = sin(t4);
t9 = cos(t2);
t10 = cos(t4);
RLeftArm = reshape([t5.*t10,t7.*t10,-t8,-t7.*t9+t5.*t6.*t8,t5.*t9+t6.*t7.*t8,t6.*t10,t6.*t7+t5.*t8.*t9,-t5.*t6+t7.*t8.*t9,t9.*t10],[3,3]);