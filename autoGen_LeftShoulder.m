function [rLeftShoulder,vLeftShoulder,omegaLeftShoulder,ILeftShoulder,RLeftShoulder] = autoGen_LeftShoulder(State)
%AUTOGEN_LEFTSHOULDER
%    [RLEFTSHOULDER,VLEFTSHOULDER,OMEGALEFTSHOULDER,ILEFTSHOULDER,RLEFTSHOULDER] = AUTOGEN_LEFTSHOULDER(ROX,ROY,ROZ,ALPHA,BETA,GAMMA,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13,Q14,Q15,Q16,ROXDOT,ROYDOT,ROZDOT,ALPHADOT,BETADOT,GAMMADOT,Q1DOT,Q2DOT,Q3DOT,Q4DOT,Q5DOT,Q6DOT,Q7DOT,Q8DOT,Q9DOT,Q10DOT,Q11DOT,Q12DOT,Q13DOT,Q14DOT,Q15DOT,Q16DOT)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    24-Oct-2018 20:28:40

rOx = State(1);                 
rOy = State(2);
rOz = State(3);
alpha = State(4);
beta = State(5);
gamma = State(6);
q1 = State(7);
q2 = State(7+1);
q3 = State(7+2);
q4 = State(7+3);
q5 = State(7+4);
q6 = State(7+5);
q7 = State(7+6);
q8 = State(7+7);
q9 = State(7+8);
q10 = State(7+9);
q11 = State(7+10);
q12 = State(7+11);
q13 = State(7+12);
q14 = State(7+13);
q15 = State(7+14);
q16 = State(7+15);

rOxdot = State(1 + 22);                 
rOydot = State(2+ 22);
rOzdot = State(3+ 22);
alphadot = State(4+ 22);
betadot = State(5+ 22);
gammadot = State(6+ 22);
q1dot = State(7+ 22);
q2dot = State(7+1+ 22);
q3dot = State(7+2+ 22);
q4dot = State(7+3+ 22);
q5dot = State(7+4+ 22);
q6dot = State(7+5+ 22);
q7dot = State(7+6+ 22);
q8dot = State(7+7+ 22);
q9dot = State(7+8+ 22);
q10dot = State(7+9+ 22);
q11dot = State(7+10+ 22);
q12dot = State(7+11+ 22);
q13dot = State(7+12+ 22);
q14dot = State(7+13+ 22);
q15dot = State(7+14+ 22);
q16dot = State(7+15+ 22);

t2 = cos(alpha);
t3 = sin(alpha);
t4 = cos(gamma);
t5 = sin(beta);
t6 = sin(gamma);
t7 = pi.*(1.0./1.8e1);
t8 = beta+t7;
t9 = cos(t8);
t10 = cos(beta);
t11 = t3.*t4.*t5.*3.18887e-2;
t12 = sin(t8);
t13 = t3.*t6.*3.18887e-2;
t14 = t3.*t4.*1.64721e-3;
t15 = t2.*t4.*t5.*3.18887e-2;
rLeftShoulder = [rOx+t13+t14+t15-t2.*t9.*4.23081e-3-t2.*t10.*6.272722e-2-t2.*t5.*t6.*1.64721e-3;rOy+t11-t2.*t4.*1.64721e-3-t2.*t6.*3.18887e-2-t3.*t9.*4.23081e-3-t3.*t10.*6.272722e-2-t3.*t5.*t6.*1.64721e-3;rOz+t5.*6.272722e-2+t12.*4.23081e-3+t4.*t10.*3.18887e-2-t6.*t10.*1.64721e-3];
if nargout > 1
    vLeftShoulder = [rOxdot+alphadot.*(-t11+t2.*t4.*1.64721e-3+t2.*t6.*3.18887e-2+t3.*t9.*4.23081e-3+t3.*t10.*6.272722e-2+t3.*t5.*t6.*1.64721e-3)+betadot.*(t2.*t5.*6.272722e-2+t2.*t12.*4.23081e-3+t2.*t4.*t10.*3.18887e-2-t2.*t6.*t10.*1.64721e-3)-gammadot.*(t3.*t4.*(-3.18887e-2)+t3.*t6.*1.64721e-3+t2.*t4.*t5.*1.64721e-3+t2.*t5.*t6.*3.18887e-2);rOydot+betadot.*(t3.*t5.*6.272722e-2+t3.*t12.*4.23081e-3+t3.*t4.*t10.*3.18887e-2-t3.*t6.*t10.*1.64721e-3)+alphadot.*(t13+t14+t15-t2.*t9.*4.23081e-3-t2.*t10.*6.272722e-2-t2.*t5.*t6.*1.64721e-3)-gammadot.*(t2.*t4.*3.18887e-2-t2.*t6.*1.64721e-3+t3.*t4.*t5.*1.64721e-3+t3.*t5.*t6.*3.18887e-2);rOzdot-gammadot.*(t4.*t10.*1.64721e-3+t6.*t10.*3.18887e-2)+betadot.*(t9.*4.23081e-3+t10.*6.272722e-2-t4.*t5.*3.18887e-2+t5.*t6.*1.64721e-3)];
end
if nargout > 2
    t16 = gamma-q1;
    t17 = sin(t16);
    t18 = cos(t16);
    omegaLeftShoulder = [gammadot-q1dot-alphadot.*t12;betadot.*t18+alphadot.*t9.*t17;-betadot.*t17+alphadot.*t9.*t18];
end
if nargout > 3
    ILeftShoulder = reshape([3.6e-6,0.0,0.0,0.0,7.698e-7,0.0,0.0,0.0,3.6e-6],[3,3]);
end
if nargout > 4
    RLeftShoulder = reshape([t2.*t9,t3.*t9,-t12,-t3.*t18+t2.*t12.*t17,t2.*t18+t3.*t12.*t17,t9.*t17,t3.*t17+t2.*t12.*t18,-t2.*t17+t3.*t12.*t18,t9.*t18],[3,3]);
end
