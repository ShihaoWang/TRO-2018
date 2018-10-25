function [rLeftHand,vLeftHand,omegaLeftHand,ILeftHand,RLeftHand] = autoGen_LeftHand(State)
%AUTOGEN_LEFTHAND
%    [RLEFTHAND,VLEFTHAND,OMEGALEFTHAND,ILEFTHAND,RLEFTHAND] = AUTOGEN_LEFTHAND(ROX,ROY,ROZ,ALPHA,BETA,GAMMA,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13,Q14,Q15,Q16,ROXDOT,ROYDOT,ROZDOT,ALPHADOT,BETADOT,GAMMADOT,Q1DOT,Q2DOT,Q3DOT,Q4DOT,Q5DOT,Q6DOT,Q7DOT,Q8DOT,Q9DOT,Q10DOT,Q11DOT,Q12DOT,Q13DOT,Q14DOT,Q15DOT,Q16DOT)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    24-Oct-2018 20:28:42

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

t2 = sin(alpha);
t3 = pi.*(1.0./1.8e1);
t4 = cos(alpha);
t5 = gamma-q1;
t6 = beta+t3;
t7 = cos(gamma);
t8 = sin(beta);
t9 = sin(gamma);
t10 = sin(t5);
t11 = beta+q2+q3+t3;
t12 = cos(t6);
t13 = cos(t5);
t14 = beta+q2+t3;
t15 = cos(t14);
t16 = cos(t11);
t17 = cos(beta);
t18 = sin(t11);
t19 = sin(t6);
t20 = sin(t14);
t21 = t4.*t18.*4.614073e-2;
t22 = t2.*t10.*3.939380000000001e-3;
t23 = t4.*t13.*t18.*2.968938e-2;
t24 = t2.*t10.*t19.*2.575e-2;
t25 = t2.*t7.*t8.*3.18887e-2;
t26 = t4.*t20.*(9.0./1.25e2);
t27 = t2.*t9.*3.18887e-2;
t28 = t2.*t13.*3.939380000000001e-3;
t29 = t2.*t7.*1.64721e-3;
t30 = t4.*t10.*t19.*2.575e-2;
t31 = t4.*t7.*t8.*3.18887e-2;
rLeftHand = [rOx+t27+t28+t29+t30+t31-t4.*t12.*1.950677e-2-t4.*t15.*(9.0./1.25e2)-t4.*t16.*4.614073e-2-t4.*t17.*6.272722e-2-t4.*t8.*t9.*1.64721e-3-t4.*t10.*t18.*2.968938e-2;rOy+t24+t25-t4.*t7.*1.64721e-3-t4.*t9.*3.18887e-2-t2.*t12.*1.950677e-2-t2.*t15.*(9.0./1.25e2)-t4.*t13.*3.939380000000001e-3-t2.*t16.*4.614073e-2-t2.*t17.*6.272722e-2-t2.*t8.*t9.*1.64721e-3-t2.*t10.*t18.*2.968938e-2;rOz+t8.*6.272722e-2+t18.*4.614073e-2+t19.*1.950677e-2+t20.*(9.0./1.25e2)+t10.*t12.*2.575e-2+t7.*t17.*3.18887e-2-t9.*t17.*1.64721e-3-t10.*t16.*2.968938e-2];
if nargout > 1
    t32 = t4.*t10.*3.939380000000001e-3;
    t33 = t2.*t13.*t19.*2.575e-2;
    t34 = t2.*t18.*4.614073e-2;
    t35 = t2.*t20.*(9.0./1.25e2);
    t36 = t16.*4.614073e-2;
    t37 = t10.*t18.*2.968938e-2;
    t38 = t12.*t13.*2.575e-2;
    t39 = t15.*(9.0./1.25e2);
    vLeftHand = [rOxdot+alphadot.*(-t24-t25+t4.*t7.*1.64721e-3+t4.*t9.*3.18887e-2+t2.*t12.*1.950677e-2+t2.*t15.*(9.0./1.25e2)+t4.*t13.*3.939380000000001e-3+t2.*t16.*4.614073e-2+t2.*t17.*6.272722e-2+t2.*t8.*t9.*1.64721e-3+t2.*t10.*t18.*2.968938e-2)-gammadot.*(t22+t23-t2.*t7.*3.18887e-2+t2.*t9.*1.64721e-3+t4.*t7.*t8.*1.64721e-3+t4.*t8.*t9.*3.18887e-2-t4.*t13.*t19.*2.575e-2)+q3dot.*(t21-t4.*t10.*t16.*2.968938e-2)+betadot.*(t21+t26+t4.*t8.*6.272722e-2+t4.*t19.*1.950677e-2+t4.*t10.*t12.*2.575e-2+t4.*t7.*t17.*3.18887e-2-t4.*t9.*t17.*1.64721e-3-t4.*t10.*t16.*2.968938e-2)+q1dot.*(t22+t23-t4.*t13.*t19.*2.575e-2)+q2dot.*(t21+t26-t4.*t10.*t16.*2.968938e-2);rOydot-gammadot.*(-t32-t33+t4.*t7.*3.18887e-2-t4.*t9.*1.64721e-3+t2.*t7.*t8.*1.64721e-3+t2.*t8.*t9.*3.18887e-2+t2.*t13.*t18.*2.968938e-2)+q3dot.*(t34-t2.*t10.*t16.*2.968938e-2)+betadot.*(t34+t35+t2.*t8.*6.272722e-2+t2.*t19.*1.950677e-2+t2.*t10.*t12.*2.575e-2+t2.*t7.*t17.*3.18887e-2-t2.*t9.*t17.*1.64721e-3-t2.*t10.*t16.*2.968938e-2)-alphadot.*(-t27-t28-t29-t30-t31+t4.*t12.*1.950677e-2+t4.*t15.*(9.0./1.25e2)+t4.*t16.*4.614073e-2+t4.*t17.*6.272722e-2+t4.*t8.*t9.*1.64721e-3+t4.*t10.*t18.*2.968938e-2)-q1dot.*(t32+t33-t2.*t13.*t18.*2.968938e-2)+q2dot.*(t34+t35-t2.*t10.*t16.*2.968938e-2);rOzdot-q1dot.*(t38-t13.*t16.*2.968938e-2)+q3dot.*(t36+t37)-gammadot.*(-t38+t7.*t17.*1.64721e-3+t9.*t17.*3.18887e-2+t13.*t16.*2.968938e-2)+q2dot.*(t36+t37+t39)+betadot.*(t12.*1.950677e-2+t17.*6.272722e-2+t36+t37+t39-t7.*t8.*3.18887e-2+t8.*t9.*1.64721e-3-t10.*t19.*2.575e-2)];
end
if nargout > 2
    t40 = betadot+q2dot+q3dot;
    omegaLeftHand = [gammadot-q1dot-alphadot.*t18;t13.*t40+alphadot.*t10.*t16;-t10.*t40+alphadot.*t13.*t16];
end
if nargout > 3
    ILeftHand = reshape([9.0e-6,0.0,0.0,0.0,4.0e-5,0.0,0.0,0.0,4.0e-5],[3,3]);
end
if nargout > 4
    RLeftHand = reshape([t4.*t16,t2.*t16,-t18,-t2.*t13+t4.*t10.*t18,t4.*t13+t2.*t10.*t18,t10.*t16,t2.*t10+t4.*t13.*t18,-t4.*t10+t2.*t13.*t18,t13.*t16],[3,3]);
end
