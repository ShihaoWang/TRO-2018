function [rLeftAnkle,vLeftAnkle,omegaLeftAnkle,ILeftAnkle,RLeftAnkle] = autoGen_LeftAnkle(rOx,rOy,rOz,alpha,beta,gamma,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,rOxdot,rOydot,rOzdot,alphadot,betadot,gammadot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot,q11dot,q12dot,q13dot,q14dot,q15dot,q16dot)
%AUTOGEN_LEFTANKLE
%    [RLEFTANKLE,VLEFTANKLE,OMEGALEFTANKLE,ILEFTANKLE,RLEFTANKLE] = AUTOGEN_LEFTANKLE(ROX,ROY,ROZ,ALPHA,BETA,GAMMA,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13,Q14,Q15,Q16,ROXDOT,ROYDOT,ROZDOT,ALPHADOT,BETADOT,GAMMADOT,Q1DOT,Q2DOT,Q3DOT,Q4DOT,Q5DOT,Q6DOT,Q7DOT,Q8DOT,Q9DOT,Q10DOT,Q11DOT,Q12DOT,Q13DOT,Q14DOT,Q15DOT,Q16DOT)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    24-Oct-2018 20:28:51

t2 = sin(alpha);
t3 = -gamma+q13+q14+q15;
t4 = cos(alpha);
t5 = beta-q12;
t6 = sin(gamma);
t7 = sin(t5);
t8 = cos(t3);
t9 = sin(t3);
t10 = gamma-q13;
t11 = -gamma+q13+q14;
t12 = cos(gamma);
t13 = sin(beta);
t14 = cos(t5);
t15 = sin(t10);
t16 = sin(t11);
t17 = cos(beta);
t18 = cos(t10);
t19 = cos(t11);
t20 = t2.*t8.*3.40719e-2;
t21 = t2.*t9.*6.88525e-3;
t22 = t4.*t7.*t9.*3.40719e-2;
t23 = t2.*t19.*(3.3e1./5.0e2);
t24 = t4.*t7.*t16.*(3.3e1./5.0e2);
t25 = t4.*t9.*t14.*6.88525e-3;
t26 = t4.*t14.*t18.*5.55e-2;
t27 = t4.*t14.*t19.*(3.3e1./5.0e2);
t28 = t4.*t12.*t14.*(7.0./2.0e2);
t29 = t4.*t6.*t14.*3.225e-2;
t30 = t4.*t8.*t14.*3.40719e-2;
t31 = t4.*t8.*6.88525e-3;
t32 = t4.*t15.*5.55e-2;
t33 = t4.*t6.*1.2766659e-1;
t34 = t2.*t6.*t13.*2.330279e-2;
t35 = t2.*t18.*5.55e-2;
t36 = t4.*t8.*3.40719e-2;
t37 = t4.*t9.*6.88525e-3;
t38 = t2.*t7.*t8.*6.88525e-3;
t39 = t2.*t9.*3.40719e-2;
t40 = t2.*t16.*(3.3e1./5.0e2);
t41 = t2.*t12.*8.94721e-3;
t42 = t4.*t6.*t13.*2.330279e-2;
rLeftAnkle = [rOx+t39+t40+t41+t42-t2.*t6.*1.2766659e-1-t2.*t8.*6.88525e-3-t2.*t15.*5.55e-2-t4.*t14.*1.040533e-2-t4.*t17.*3.172834e-2-t4.*t6.*t7.*3.225e-2-t4.*t7.*t8.*3.40719e-2-t4.*t7.*t9.*6.88525e-3-t4.*t7.*t12.*(7.0./2.0e2)-t4.*t7.*t18.*5.55e-2-t4.*t12.*t13.*9.266658999999999e-2-t4.*t7.*t19.*(3.3e1./5.0e2);rOy+t31+t32+t33+t34-t4.*t9.*3.40719e-2-t2.*t14.*1.040533e-2-t4.*t12.*8.94721e-3-t2.*t17.*3.172834e-2-t4.*t16.*(3.3e1./5.0e2)-t2.*t6.*t7.*3.225e-2-t2.*t7.*t8.*3.40719e-2-t2.*t7.*t9.*6.88525e-3-t2.*t7.*t12.*(7.0./2.0e2)-t2.*t7.*t18.*5.55e-2-t2.*t12.*t13.*9.266658999999999e-2-t2.*t7.*t19.*(3.3e1./5.0e2);rOz+t7.*1.040533e-2+t13.*3.172834e-2-t6.*t14.*3.225e-2-t8.*t14.*3.40719e-2+t6.*t17.*2.330279e-2-t9.*t14.*6.88525e-3-t12.*t14.*(7.0./2.0e2)-t12.*t17.*9.266658999999999e-2-t14.*t18.*5.55e-2-t14.*t19.*(3.3e1./5.0e2)];
if nargout > 1
    t43 = t2.*t6.*t14.*3.225e-2;
    t44 = t2.*t8.*t14.*3.40719e-2;
    t45 = t2.*t9.*t14.*6.88525e-3;
    t46 = t2.*t14.*t18.*5.55e-2;
    t47 = t2.*t14.*t19.*(3.3e1./5.0e2);
    t48 = t2.*t12.*t14.*(7.0./2.0e2);
    t49 = t4.*t19.*(3.3e1./5.0e2);
    t50 = t4.*t18.*5.55e-2;
    t51 = t2.*t7.*t15.*5.55e-2;
    t52 = t14.*1.040533e-2;
    t53 = t7.*t12.*(7.0./2.0e2);
    t54 = t6.*t7.*3.225e-2;
    t55 = t7.*t8.*3.40719e-2;
    t56 = t7.*t9.*6.88525e-3;
    t57 = t7.*t18.*5.55e-2;
    t58 = t7.*t19.*(3.3e1./5.0e2);
    t59 = t9.*t14.*3.40719e-2;
    t60 = t14.*t16.*(3.3e1./5.0e2);
    t61 = t8.*t14.*6.88525e-3;
    t62 = t14.*t15.*5.55e-2;
    vLeftAnkle = [rOxdot-gammadot.*(t20+t21+t22+t23+t24+t35+t2.*t6.*8.94721e-3+t2.*t12.*1.2766659e-1-t4.*t6.*t7.*(7.0./2.0e2)-t4.*t7.*t8.*6.88525e-3-t4.*t6.*t13.*9.266658999999999e-2+t4.*t7.*t12.*3.225e-2-t4.*t7.*t15.*5.55e-2-t4.*t12.*t13.*2.330279e-2)+q14dot.*(t20+t21+t22+t23+t24-t4.*t7.*t8.*6.88525e-3)-betadot.*(t25+t26+t27+t28+t29+t30-t4.*t7.*1.040533e-2-t4.*t13.*3.172834e-2-t4.*t6.*t17.*2.330279e-2+t4.*t12.*t17.*9.266658999999999e-2)+q15dot.*(t20+t21+t22-t4.*t7.*t8.*6.88525e-3)+q13dot.*(t20+t21+t22+t23+t24+t35-t4.*t7.*t8.*6.88525e-3-t4.*t7.*t15.*5.55e-2)+alphadot.*(-t31-t32-t33-t34+t4.*t9.*3.40719e-2+t2.*t14.*1.040533e-2+t4.*t12.*8.94721e-3+t2.*t17.*3.172834e-2+t4.*t16.*(3.3e1./5.0e2)+t2.*t6.*t7.*3.225e-2+t2.*t7.*t8.*3.40719e-2+t2.*t7.*t9.*6.88525e-3+t2.*t7.*t12.*(7.0./2.0e2)+t2.*t7.*t18.*5.55e-2+t2.*t12.*t13.*9.266658999999999e-2+t2.*t7.*t19.*(3.3e1./5.0e2))+q12dot.*(t25+t26+t27+t28+t29+t30-t4.*t7.*1.040533e-2);rOydot-q14dot.*(t36+t37+t38+t49-t2.*t7.*t9.*3.40719e-2-t2.*t7.*t16.*(3.3e1./5.0e2))-betadot.*(t43+t44+t45+t46+t47+t48-t2.*t7.*1.040533e-2-t2.*t13.*3.172834e-2-t2.*t6.*t17.*2.330279e-2+t2.*t12.*t17.*9.266658999999999e-2)+gammadot.*(t36+t37+t38+t49+t50+t51+t4.*t6.*8.94721e-3+t4.*t12.*1.2766659e-1+t2.*t6.*t7.*(7.0./2.0e2)-t2.*t7.*t9.*3.40719e-2+t2.*t6.*t13.*9.266658999999999e-2-t2.*t7.*t12.*3.225e-2-t2.*t7.*t16.*(3.3e1./5.0e2)+t2.*t12.*t13.*2.330279e-2)+q12dot.*(t43+t44+t45+t46+t47+t48-t2.*t7.*1.040533e-2)-q15dot.*(t36+t37+t38-t2.*t7.*t9.*3.40719e-2)-alphadot.*(-t39-t40-t41-t42+t2.*t6.*1.2766659e-1+t2.*t8.*6.88525e-3+t2.*t15.*5.55e-2+t4.*t14.*1.040533e-2+t4.*t17.*3.172834e-2+t4.*t6.*t7.*3.225e-2+t4.*t7.*t8.*3.40719e-2+t4.*t7.*t9.*6.88525e-3+t4.*t7.*t12.*(7.0./2.0e2)+t4.*t7.*t18.*5.55e-2+t4.*t12.*t13.*9.266658999999999e-2+t4.*t7.*t19.*(3.3e1./5.0e2))-q13dot.*(t36+t37+t38+t49+t50+t51-t2.*t7.*t9.*3.40719e-2-t2.*t7.*t16.*(3.3e1./5.0e2));rOzdot+q15dot.*(t59-t8.*t14.*6.88525e-3)-q12dot.*(t52+t53+t54+t55+t56+t57+t58)+gammadot.*(-t59-t60+t61+t62+t6.*t14.*(7.0./2.0e2)+t6.*t17.*9.266658999999999e-2-t12.*t14.*3.225e-2+t12.*t17.*2.330279e-2)+q14dot.*(t59+t60-t8.*t14.*6.88525e-3)+q13dot.*(t59+t60-t61-t62)+betadot.*(t17.*3.172834e-2+t52+t53+t54+t55+t56+t57+t58-t6.*t13.*2.330279e-2+t12.*t13.*9.266658999999999e-2)];
end
if nargout > 2
    t63 = betadot-q12dot;
    omegaLeftAnkle = [gammadot-q13dot-q14dot-q15dot-alphadot.*t7;t8.*t63-alphadot.*t9.*t14;t9.*t63+alphadot.*t8.*t14];
end
if nargout > 3
    ILeftAnkle = reshape([3.0e-5,0.0,0.0,0.0,3.0e-5,0.0,0.0,0.0,3.0e-5],[3,3]);
end
if nargout > 4
    RLeftAnkle = reshape([t4.*t14,t2.*t14,-t7,-t2.*t8-t4.*t7.*t9,t4.*t8-t2.*t7.*t9,-t9.*t14,-t2.*t9+t4.*t7.*t8,t4.*t9+t2.*t7.*t8,t8.*t14],[3,3]);
end