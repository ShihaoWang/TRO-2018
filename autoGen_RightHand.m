function [rRightHand,vRightHand,omegaRightHand,IRightHand,RRightHand] = autoGen_RightHand(rOx,rOy,rOz,alpha,beta,gamma,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,rOxdot,rOydot,rOzdot,alphadot,betadot,gammadot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot,q11dot,q12dot,q13dot,q14dot,q15dot,q16dot)
%AUTOGEN_RIGHTHAND
%    [RRIGHTHAND,VRIGHTHAND,OMEGARIGHTHAND,IRIGHTHAND,RRIGHTHAND] = AUTOGEN_RIGHTHAND(ROX,ROY,ROZ,ALPHA,BETA,GAMMA,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13,Q14,Q15,Q16,ROXDOT,ROYDOT,ROZDOT,ALPHADOT,BETADOT,GAMMADOT,Q1DOT,Q2DOT,Q3DOT,Q4DOT,Q5DOT,Q6DOT,Q7DOT,Q8DOT,Q9DOT,Q10DOT,Q11DOT,Q12DOT,Q13DOT,Q14DOT,Q15DOT,Q16DOT)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    24-Oct-2018 20:28:45

t2 = sin(alpha);
t3 = cos(alpha);
t4 = gamma+q9;
t7 = pi.*(1.0./1.8e1);
t5 = beta+q10+q11-t7;
t6 = sin(t4);
t8 = cos(gamma);
t9 = sin(beta);
t10 = sin(gamma);
t11 = beta-t7;
t12 = cos(t4);
t13 = cos(t5);
t14 = cos(beta);
t15 = sin(t5);
t16 = sin(t11);
t17 = beta+q10-t7;
t18 = cos(t11);
t19 = sin(t17);
t20 = t3.*t19.*(9.0./1.25e2);
t21 = t3.*t15.*4.614073e-2;
t22 = t3.*t6.*t13.*2.968937e-2;
t23 = t2.*t6.*3.93937e-3;
t24 = t3.*t12.*t15.*2.968937e-2;
t25 = t2.*t18.*1.950677e-2;
t26 = cos(t17);
t27 = t2.*t26.*(9.0./1.25e2);
t28 = t2.*t13.*4.614073e-2;
t29 = t2.*t14.*6.272722e-2;
t30 = t2.*t6.*t16.*2.575e-2;
t31 = t2.*t8.*t9.*3.18887e-2;
t32 = t2.*t19.*(9.0./1.25e2);
t33 = t2.*t15.*4.614073e-2;
t34 = t2.*t6.*t13.*2.968937e-2;
t35 = t2.*t10.*3.18887e-2;
t36 = t2.*t12.*3.93937e-3;
t37 = t3.*t13.*4.614073e-2;
t38 = t3.*t14.*6.272722e-2;
t39 = t2.*t8.*1.64721e-3;
t40 = t3.*t6.*t16.*2.575e-2;
t41 = t3.*t8.*t9.*3.18887e-2;
rRightHand = [rOx+t35+t36+t37+t38+t39+t40+t41+t3.*cos(beta+q10-pi.*(1.0./1.8e1)).*(9.0./1.25e2)+t3.*cos(beta-pi.*(1.0./1.8e1)).*1.950677e-2-t3.*t9.*t10.*1.64721e-3-t3.*t6.*t15.*2.968937e-2;rOy+t25+t27+t28+t29+t30+t31-t3.*t8.*1.64721e-3-t3.*t10.*3.18887e-2-t3.*t12.*3.93937e-3-t2.*t9.*t10.*1.64721e-3-t2.*t6.*t15.*2.968937e-2;rOz-t9.*6.272722e-2-t15.*4.614073e-2-t16.*1.950677e-2-t19.*(9.0./1.25e2)-t6.*t13.*2.968937e-2+t8.*t14.*3.18887e-2+t6.*t18.*2.575e-2-t10.*t14.*1.64721e-3];
if nargout > 1
    t42 = t3.*t6.*3.93937e-3;
    t43 = t2.*t12.*t16.*2.575e-2;
    t44 = t13.*4.614073e-2;
    t45 = t12.*t13.*2.968937e-2;
    t46 = t26.*(9.0./1.25e2);
    vRightHand = [rOxdot-q11dot.*(t21+t22)-betadot.*(t20+t21+t22+t3.*t9.*6.272722e-2+t3.*t16.*1.950677e-2-t3.*t8.*t14.*3.18887e-2-t3.*t6.*t18.*2.575e-2+t3.*t10.*t14.*1.64721e-3)-gammadot.*(t23+t24-t2.*t8.*3.18887e-2+t2.*t10.*1.64721e-3+t3.*t8.*t9.*1.64721e-3+t3.*t9.*t10.*3.18887e-2-t3.*t12.*t16.*2.575e-2)-q10dot.*(t20+t21+t22)-q9dot.*(t23+t24-t3.*t12.*t16.*2.575e-2)-alphadot.*(t25+t27+t28+t29+t30+t31-t3.*t8.*1.64721e-3-t3.*t10.*3.18887e-2-t3.*t12.*3.93937e-3-t2.*t9.*t10.*1.64721e-3-t2.*t6.*t15.*2.968937e-2);rOydot-q11dot.*(t33+t34)-gammadot.*(-t42-t43+t3.*t8.*3.18887e-2-t3.*t10.*1.64721e-3+t2.*t8.*t9.*1.64721e-3+t2.*t9.*t10.*3.18887e-2+t2.*t12.*t15.*2.968937e-2)-betadot.*(t32+t33+t34+t2.*t9.*6.272722e-2+t2.*t16.*1.950677e-2-t2.*t8.*t14.*3.18887e-2-t2.*t6.*t18.*2.575e-2+t2.*t10.*t14.*1.64721e-3)-q10dot.*(t32+t33+t34)+q9dot.*(t42+t43-t2.*t12.*t15.*2.968937e-2)+alphadot.*(t35+t36+t37+t38+t39+t40+t41+t3.*t18.*1.950677e-2+t3.*t26.*(9.0./1.25e2)-t3.*t9.*t10.*1.64721e-3-t3.*t6.*t15.*2.968937e-2);rOzdot-q10dot.*(t44+t46-t6.*t15.*2.968937e-2)-betadot.*(t14.*6.272722e-2+t18.*1.950677e-2+t44+t46+t8.*t9.*3.18887e-2-t9.*t10.*1.64721e-3-t6.*t15.*2.968937e-2+t6.*t16.*2.575e-2)-gammadot.*(t45+t8.*t14.*1.64721e-3+t10.*t14.*3.18887e-2-t12.*t18.*2.575e-2)-q11dot.*(t44-t6.*t15.*2.968937e-2)-q9dot.*(t45-t12.*t18.*2.575e-2)];
end
if nargout > 2
    t47 = betadot+q10dot+q11dot;
    omegaRightHand = [gammadot+q9dot-alphadot.*t15;t12.*t47+alphadot.*t6.*t13;-t6.*t47+alphadot.*t12.*t13];
end
if nargout > 3
    IRightHand = reshape([9.0e-6,0.0,0.0,0.0,4.0e-5,0.0,0.0,0.0,4.0e-5],[3,3]);
end
if nargout > 4
    RRightHand = reshape([t3.*t13,t2.*t13,-t15,-t2.*t12+t3.*t6.*t15,t3.*t12+t2.*t6.*t15,t6.*t13,t2.*t6+t3.*t12.*t15,-t3.*t6+t2.*t12.*t15,t12.*t13],[3,3]);
end