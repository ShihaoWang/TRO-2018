function [rRightAnkle,vRightAnkle,omegaRightAnkle,IRightAnkle,RRightAnkle] = autoGen_RightAnkle(State)
%AUTOGEN_RIGHTANKLE
%    [RRIGHTANKLE,VRIGHTANKLE,OMEGARIGHTANKLE,IRIGHTANKLE,RRIGHTANKLE] = AUTOGEN_RIGHTANKLE(ROX,ROY,ROZ,ALPHA,BETA,GAMMA,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13,Q14,Q15,Q16,ROXDOT,ROYDOT,ROZDOT,ALPHADOT,BETADOT,GAMMADOT,Q1DOT,Q2DOT,Q3DOT,Q4DOT,Q5DOT,Q6DOT,Q7DOT,Q8DOT,Q9DOT,Q10DOT,Q11DOT,Q12DOT,Q13DOT,Q14DOT,Q15DOT,Q16DOT)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    24-Oct-2018 20:28:59

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
t3 = gamma+q5+q6-q7;
t4 = cos(alpha);
t5 = beta-q4;
t6 = sin(gamma);
t7 = sin(t5);
t8 = cos(t3);
t9 = sin(t3);
t10 = gamma+q5+q6;
t11 = cos(gamma);
t12 = gamma+q5;
t13 = sin(beta);
t14 = cos(t5);
t15 = sin(t10);
t16 = sin(t12);
t17 = cos(beta);
t18 = cos(t10);
t19 = cos(t12);
t20 = t2.*t9.*6.88525e-3;
t21 = t4.*t7.*t8.*6.88525e-3;
t22 = t4.*t7.*t9.*3.40719e-2;
t23 = t4.*t8.*6.88525e-3;
t24 = t4.*t9.*3.40719e-2;
t25 = t2.*t14.*1.040533e-2;
t26 = t4.*t15.*(3.3e1./5.0e2);
t27 = t4.*t16.*5.55e-2;
t28 = t2.*t17.*3.172834e-2;
t29 = t4.*t6.*1.2766659e-1;
t30 = t2.*t7.*t9.*6.88525e-3;
t31 = t2.*t6.*t13.*2.330279e-2;
t32 = t4.*t7.*t15.*(3.3e1./5.0e2);
t33 = t4.*t7.*t16.*5.55e-2;
t34 = t4.*t7.*1.040533e-2;
t35 = t4.*t14.*t18.*(3.3e1./5.0e2);
t36 = t4.*t14.*t19.*5.55e-2;
t37 = t4.*t11.*t14.*(7.0./2.0e2);
t38 = t4.*t6.*t14.*3.225e-2;
t39 = t4.*t8.*t14.*3.40719e-2;
t40 = t4.*t14.*1.040533e-2;
t41 = t4.*t17.*3.172834e-2;
t42 = t2.*t11.*8.94721e-3;
t43 = t4.*t7.*t9.*6.88525e-3;
t44 = t4.*t6.*t13.*2.330279e-2;
rRightAnkle = [rOx+t40+t41+t42+t43+t44-t2.*t6.*1.2766659e-1-t2.*t8.*6.88525e-3-t2.*t9.*3.40719e-2-t2.*t15.*(3.3e1./5.0e2)-t2.*t16.*5.55e-2-t4.*t6.*t7.*3.225e-2-t4.*t7.*t8.*3.40719e-2-t4.*t7.*t11.*(7.0./2.0e2)-t4.*t11.*t13.*9.266658999999999e-2-t4.*t7.*t18.*(3.3e1./5.0e2)-t4.*t7.*t19.*5.55e-2;rOy+t23+t24+t25+t26+t27+t28+t29+t30+t31-t4.*t11.*8.94721e-3-t2.*t6.*t7.*3.225e-2-t2.*t7.*t8.*3.40719e-2-t2.*t7.*t11.*(7.0./2.0e2)-t2.*t11.*t13.*9.266658999999999e-2-t2.*t7.*t18.*(3.3e1./5.0e2)-t2.*t7.*t19.*5.55e-2;rOz-t7.*1.040533e-2-t13.*3.172834e-2-t6.*t14.*3.225e-2-t8.*t14.*3.40719e-2+t6.*t17.*2.330279e-2+t9.*t14.*6.88525e-3-t11.*t14.*(7.0./2.0e2)-t11.*t17.*9.266658999999999e-2-t14.*t18.*(3.3e1./5.0e2)-t14.*t19.*5.55e-2];
if nargout > 1
    t45 = t4.*t8.*3.40719e-2;
    t46 = t2.*t7.*t8.*6.88525e-3;
    t47 = t2.*t7.*t9.*3.40719e-2;
    t48 = t4.*t18.*(3.3e1./5.0e2);
    t49 = t4.*t19.*5.55e-2;
    t50 = t2.*t7.*t15.*(3.3e1./5.0e2);
    t51 = t2.*t7.*t16.*5.55e-2;
    t52 = t2.*t7.*1.040533e-2;
    t53 = t2.*t6.*t14.*3.225e-2;
    t54 = t2.*t8.*t14.*3.40719e-2;
    t55 = t2.*t14.*t18.*(3.3e1./5.0e2);
    t56 = t2.*t14.*t19.*5.55e-2;
    t57 = t2.*t11.*t14.*(7.0./2.0e2);
    t58 = t7.*t19.*5.55e-2;
    t59 = t7.*t11.*(7.0./2.0e2);
    t60 = t6.*t7.*3.225e-2;
    t61 = t7.*t8.*3.40719e-2;
    t62 = t7.*t18.*(3.3e1./5.0e2);
    t63 = t8.*t14.*6.88525e-3;
    t64 = t9.*t14.*3.40719e-2;
    t65 = t14.*t15.*(3.3e1./5.0e2);
    t66 = t14.*t16.*5.55e-2;
    vRightAnkle = [rOxdot+q6dot.*(t20+t21+t22+t32-t2.*t8.*3.40719e-2-t2.*t18.*(3.3e1./5.0e2))-q7dot.*(t20+t21+t22-t2.*t8.*3.40719e-2)+gammadot.*(t20+t21+t22+t32+t33-t2.*t6.*8.94721e-3-t2.*t8.*3.40719e-2-t2.*t11.*1.2766659e-1-t2.*t18.*(3.3e1./5.0e2)-t2.*t19.*5.55e-2+t4.*t6.*t7.*(7.0./2.0e2)-t4.*t7.*t11.*3.225e-2+t4.*t6.*t13.*9.266658999999999e-2+t4.*t11.*t13.*2.330279e-2)-betadot.*(t34+t35+t36+t37+t38+t39+t4.*t13.*3.172834e-2-t4.*t6.*t17.*2.330279e-2-t4.*t9.*t14.*6.88525e-3+t4.*t11.*t17.*9.266658999999999e-2)-alphadot.*(t23+t24+t25+t26+t27+t28+t29+t30+t31-t4.*t11.*8.94721e-3-t2.*t6.*t7.*3.225e-2-t2.*t7.*t8.*3.40719e-2-t2.*t7.*t11.*(7.0./2.0e2)-t2.*t11.*t13.*9.266658999999999e-2-t2.*t7.*t18.*(3.3e1./5.0e2)-t2.*t7.*t19.*5.55e-2)+q4dot.*(t34+t35+t36+t37+t38+t39-t4.*t9.*t14.*6.88525e-3)+q5dot.*(t20+t21+t22+t32+t33-t2.*t8.*3.40719e-2-t2.*t18.*(3.3e1./5.0e2)-t2.*t19.*5.55e-2);rOydot-q7dot.*(t45+t46+t47-t4.*t9.*6.88525e-3)+gammadot.*(t45+t46+t47+t48+t49+t50+t51+t4.*t6.*8.94721e-3-t4.*t9.*6.88525e-3+t4.*t11.*1.2766659e-1+t2.*t6.*t7.*(7.0./2.0e2)-t2.*t7.*t11.*3.225e-2+t2.*t6.*t13.*9.266658999999999e-2+t2.*t11.*t13.*2.330279e-2)-alphadot.*(-t40-t41-t42-t43-t44+t2.*t6.*1.2766659e-1+t2.*t8.*6.88525e-3+t2.*t9.*3.40719e-2+t2.*t15.*(3.3e1./5.0e2)+t2.*t16.*5.55e-2+t4.*t6.*t7.*3.225e-2+t4.*t7.*t8.*3.40719e-2+t4.*t7.*t11.*(7.0./2.0e2)+t4.*t11.*t13.*9.266658999999999e-2+t4.*t7.*t18.*(3.3e1./5.0e2)+t4.*t7.*t19.*5.55e-2)-betadot.*(t52+t53+t54+t55+t56+t57+t2.*t13.*3.172834e-2-t2.*t6.*t17.*2.330279e-2-t2.*t9.*t14.*6.88525e-3+t2.*t11.*t17.*9.266658999999999e-2)+q5dot.*(t45+t46+t47+t48+t49+t50+t51-t4.*t9.*6.88525e-3)+q4dot.*(t52+t53+t54+t55+t56+t57-t2.*t9.*t14.*6.88525e-3)+q6dot.*(t45+t46+t47+t48+t50-t4.*t9.*6.88525e-3);rOzdot-q7dot.*(t63+t64)-q4dot.*(t14.*(-1.040533e-2)+t58+t59+t60+t61+t62-t7.*t9.*6.88525e-3)+q6dot.*(t63+t64+t65)+betadot.*(t14.*(-1.040533e-2)-t17.*3.172834e-2+t58+t59+t60+t61+t62-t7.*t9.*6.88525e-3-t6.*t13.*2.330279e-2+t11.*t13.*9.266658999999999e-2)+gammadot.*(t63+t64+t65+t66+t6.*t14.*(7.0./2.0e2)+t6.*t17.*9.266658999999999e-2-t11.*t14.*3.225e-2+t11.*t17.*2.330279e-2)+q5dot.*(t63+t64+t65+t66)];
end
if nargout > 2
    t67 = betadot-q4dot;
    omegaRightAnkle = [gammadot+q5dot+q6dot-q7dot-alphadot.*t7;t8.*t67+alphadot.*t9.*t14;-t9.*t67+alphadot.*t8.*t14];
end
if nargout > 3
    IRightAnkle = reshape([3.0e-5,0.0,0.0,0.0,3.0e-5,0.0,0.0,0.0,3.0e-5],[3,3]);
end
if nargout > 4
    RRightAnkle = reshape([t4.*t14,t2.*t14,-t7,-t2.*t8+t4.*t7.*t9,t4.*t8+t2.*t7.*t9,t9.*t14,t2.*t9+t4.*t7.*t8,-t4.*t9+t2.*t7.*t8,t8.*t14],[3,3]);
end
