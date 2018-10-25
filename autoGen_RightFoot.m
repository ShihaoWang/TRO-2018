function [rRightFoot,vRightFoot,omegaRightFoot,IRightFoot,RRightFoot] = autoGen_RightFoot(rOx,rOy,rOz,alpha,beta,gamma,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,rOxdot,rOydot,rOzdot,alphadot,betadot,gammadot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot,q11dot,q12dot,q13dot,q14dot,q15dot,q16dot)
%AUTOGEN_RIGHTFOOT
%    [RRIGHTFOOT,VRIGHTFOOT,OMEGARIGHTFOOT,IRIGHTFOOT,RRIGHTFOOT] = AUTOGEN_RIGHTFOOT(ROX,ROY,ROZ,ALPHA,BETA,GAMMA,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13,Q14,Q15,Q16,ROXDOT,ROYDOT,ROZDOT,ALPHADOT,BETADOT,GAMMADOT,Q1DOT,Q2DOT,Q3DOT,Q4DOT,Q5DOT,Q6DOT,Q7DOT,Q8DOT,Q9DOT,Q10DOT,Q11DOT,Q12DOT,Q13DOT,Q14DOT,Q15DOT,Q16DOT)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    24-Oct-2018 20:29:02

t2 = sin(alpha);
t3 = gamma+q5+q6-q7;
t4 = cos(alpha);
t5 = beta-q4;
t6 = sin(gamma);
t7 = sin(t5);
t8 = cos(t3);
t9 = sin(t3);
t10 = gamma+q5+q6;
t11 = beta-q4+q8;
t12 = sin(t11);
t13 = cos(gamma);
t14 = gamma+q5;
t15 = sin(beta);
t16 = cos(t5);
t17 = sin(t10);
t18 = cos(t11);
t19 = sin(t14);
t20 = cos(beta);
t21 = cos(t10);
t22 = cos(t14);
t23 = t4.*t12.*2.0799e-2;
t24 = t4.*t8.*t18.*2.343266e-2;
t25 = t4.*t9.*t18.*2.57509e-2;
t26 = t2.*t9.*7.4991e-3;
t27 = t4.*t7.*t8.*3.325e-2;
t28 = t4.*t7.*t9.*(7.0./2.0e2);
t29 = t4.*t7.*t17.*(3.3e1./5.0e2);
t30 = t4.*t9.*t12.*2.343266e-2;
t31 = t4.*t8.*7.4991e-3;
t32 = t4.*t9.*5.843266e-2;
t33 = t2.*t16.*(1.0./1.0e3);
t34 = t4.*t17.*(3.3e1./5.0e2);
t35 = t2.*t18.*2.0799e-2;
t36 = t4.*t19.*5.55e-2;
t37 = t2.*t20.*3.172834e-2;
t38 = t4.*t6.*1.2766659e-1;
t39 = t2.*t7.*t9.*3.325e-2;
t40 = t2.*t6.*t15.*2.330279e-2;
t41 = t4.*t7.*(1.0./1.0e3);
t42 = t4.*t16.*t21.*(3.3e1./5.0e2);
t43 = t4.*t16.*t22.*5.55e-2;
t44 = t4.*t13.*t16.*(7.0./2.0e2);
t45 = t4.*t6.*t16.*3.225e-2;
t46 = t4.*t8.*t16.*(7.0./2.0e2);
t47 = t4.*t7.*t19.*5.55e-2;
t48 = t4.*t16.*(1.0./1.0e3);
t49 = t4.*t18.*2.0799e-2;
t50 = t4.*t20.*3.172834e-2;
t51 = t2.*t13.*8.94721e-3;
t52 = t4.*t7.*t9.*3.325e-2;
t53 = t4.*t6.*t15.*2.330279e-2;
rRightFoot = [rOx+t48+t49+t50+t51+t52+t53-t2.*t6.*1.2766659e-1-t2.*t8.*7.4991e-3-t2.*t9.*5.843266e-2-t2.*t17.*(3.3e1./5.0e2)-t2.*t19.*5.55e-2-t4.*t6.*t7.*3.225e-2-t4.*t7.*t8.*(7.0./2.0e2)-t4.*t7.*t13.*(7.0./2.0e2)-t4.*t8.*t12.*2.343266e-2-t4.*t9.*t12.*2.57509e-2-t4.*t7.*t21.*(3.3e1./5.0e2)-t4.*t13.*t15.*9.266658999999999e-2-t4.*t7.*t22.*5.55e-2;rOy+t31+t32+t33+t34+t35+t36+t37+t38+t39+t40-t4.*t13.*8.94721e-3-t2.*t6.*t7.*3.225e-2-t2.*t7.*t8.*(7.0./2.0e2)-t2.*t7.*t13.*(7.0./2.0e2)-t2.*t8.*t12.*2.343266e-2-t2.*t9.*t12.*2.57509e-2-t2.*t7.*t21.*(3.3e1./5.0e2)-t2.*t13.*t15.*9.266658999999999e-2-t2.*t7.*t22.*5.55e-2;rOz-t7.*(1.0./1.0e3)-t12.*2.0799e-2-t15.*3.172834e-2-t6.*t16.*3.225e-2-t8.*t16.*(7.0./2.0e2)+t9.*t16.*3.325e-2+t6.*t20.*2.330279e-2-t8.*t18.*2.343266e-2-t9.*t18.*2.57509e-2-t13.*t16.*(7.0./2.0e2)-t13.*t20.*9.266658999999999e-2-t16.*t21.*(3.3e1./5.0e2)-t16.*t22.*5.55e-2];
if nargout > 1
    t54 = t4.*t8.*5.843266e-2;
    t55 = t2.*t7.*t8.*3.325e-2;
    t56 = t2.*t7.*t9.*(7.0./2.0e2);
    t57 = t2.*t9.*t12.*2.343266e-2;
    t58 = t4.*t21.*(3.3e1./5.0e2);
    t59 = t4.*t22.*5.55e-2;
    t60 = t2.*t7.*t17.*(3.3e1./5.0e2);
    t61 = t2.*t7.*t19.*5.55e-2;
    t62 = t2.*t12.*2.0799e-2;
    t63 = t2.*t8.*t18.*2.343266e-2;
    t64 = t2.*t9.*t18.*2.57509e-2;
    t65 = t2.*t7.*(1.0./1.0e3);
    t66 = t2.*t6.*t16.*3.225e-2;
    t67 = t2.*t8.*t16.*(7.0./2.0e2);
    t68 = t2.*t16.*t21.*(3.3e1./5.0e2);
    t69 = t2.*t16.*t22.*5.55e-2;
    t70 = t2.*t13.*t16.*(7.0./2.0e2);
    t71 = t8.*t12.*2.343266e-2;
    t72 = t9.*t12.*2.57509e-2;
    t73 = t8.*t16.*3.325e-2;
    t74 = t9.*t16.*(7.0./2.0e2);
    t75 = t9.*t18.*2.343266e-2;
    t76 = t16.*t17.*(3.3e1./5.0e2);
    t77 = t16.*t19.*5.55e-2;
    t78 = t7.*t22.*5.55e-2;
    t79 = t7.*t13.*(7.0./2.0e2);
    t80 = t6.*t7.*3.225e-2;
    t81 = t7.*t8.*(7.0./2.0e2);
    t82 = t7.*t21.*(3.3e1./5.0e2);
    vRightFoot = [rOxdot+q5dot.*(t26+t27+t28+t29+t30+t47-t2.*t8.*5.843266e-2-t2.*t21.*(3.3e1./5.0e2)-t2.*t22.*5.55e-2-t4.*t8.*t12.*2.57509e-2)+q6dot.*(t26+t27+t28+t29+t30-t2.*t8.*5.843266e-2-t2.*t21.*(3.3e1./5.0e2)-t4.*t8.*t12.*2.57509e-2)-alphadot.*(t31+t32+t33+t34+t35+t36+t37+t38+t39+t40-t4.*t13.*8.94721e-3-t2.*t6.*t7.*3.225e-2-t2.*t7.*t8.*(7.0./2.0e2)-t2.*t7.*t13.*(7.0./2.0e2)-t2.*t8.*t12.*2.343266e-2-t2.*t9.*t12.*2.57509e-2-t2.*t7.*t21.*(3.3e1./5.0e2)-t2.*t13.*t15.*9.266658999999999e-2-t2.*t7.*t22.*5.55e-2)-q8dot.*(t23+t24+t25)-q7dot.*(t26+t27+t28+t30-t2.*t8.*5.843266e-2-t4.*t8.*t12.*2.57509e-2)-betadot.*(t23+t24+t25+t41+t42+t43+t44+t45+t46+t4.*t15.*3.172834e-2-t4.*t9.*t16.*3.325e-2-t4.*t6.*t20.*2.330279e-2+t4.*t13.*t20.*9.266658999999999e-2)+gammadot.*(t26+t27+t28+t29+t30+t47-t2.*t6.*8.94721e-3-t2.*t8.*5.843266e-2-t2.*t13.*1.2766659e-1-t2.*t21.*(3.3e1./5.0e2)-t2.*t22.*5.55e-2+t4.*t6.*t7.*(7.0./2.0e2)-t4.*t7.*t13.*3.225e-2-t4.*t8.*t12.*2.57509e-2+t4.*t6.*t15.*9.266658999999999e-2+t4.*t13.*t15.*2.330279e-2)+q4dot.*(t23+t24+t25+t41+t42+t43+t44+t45+t46-t4.*t9.*t16.*3.325e-2);rOydot+gammadot.*(t54+t55+t56+t57+t58+t59+t60+t61+t4.*t6.*8.94721e-3-t4.*t9.*7.4991e-3+t4.*t13.*1.2766659e-1+t2.*t6.*t7.*(7.0./2.0e2)-t2.*t7.*t13.*3.225e-2-t2.*t8.*t12.*2.57509e-2+t2.*t6.*t15.*9.266658999999999e-2+t2.*t13.*t15.*2.330279e-2)-q8dot.*(t62+t63+t64)+q5dot.*(t54+t55+t56+t57+t58+t59+t60+t61-t4.*t9.*7.4991e-3-t2.*t8.*t12.*2.57509e-2)-betadot.*(t62+t63+t64+t65+t66+t67+t68+t69+t70+t2.*t15.*3.172834e-2-t2.*t9.*t16.*3.325e-2-t2.*t6.*t20.*2.330279e-2+t2.*t13.*t20.*9.266658999999999e-2)-alphadot.*(-t48-t49-t50-t51-t52-t53+t2.*t6.*1.2766659e-1+t2.*t8.*7.4991e-3+t2.*t9.*5.843266e-2+t2.*t17.*(3.3e1./5.0e2)+t2.*t19.*5.55e-2+t4.*t6.*t7.*3.225e-2+t4.*t7.*t8.*(7.0./2.0e2)+t4.*t7.*t13.*(7.0./2.0e2)+t4.*t8.*t12.*2.343266e-2+t4.*t9.*t12.*2.57509e-2+t4.*t7.*t21.*(3.3e1./5.0e2)+t4.*t13.*t15.*9.266658999999999e-2+t4.*t7.*t22.*5.55e-2)+q6dot.*(t54+t55+t56+t57+t58+t60-t4.*t9.*7.4991e-3-t2.*t8.*t12.*2.57509e-2)-q7dot.*(t54+t55+t56+t57-t4.*t9.*7.4991e-3-t2.*t8.*t12.*2.57509e-2)+q4dot.*(t62+t63+t64+t65+t66+t67+t68+t69+t70-t2.*t9.*t16.*3.325e-2);rOzdot+q6dot.*(t73+t74+t75+t76-t8.*t18.*2.57509e-2)+q8dot.*(t18.*(-2.0799e-2)+t71+t72)+gammadot.*(t73+t74+t75+t76+t77+t6.*t16.*(7.0./2.0e2)+t6.*t20.*9.266658999999999e-2-t8.*t18.*2.57509e-2-t13.*t16.*3.225e-2+t13.*t20.*2.330279e-2)-q4dot.*(t16.*(-1.0./1.0e3)-t18.*2.0799e-2+t71+t72+t78+t79+t80+t81+t82-t7.*t9.*3.325e-2)+q5dot.*(t73+t74+t75+t76+t77-t8.*t18.*2.57509e-2)-q7dot.*(t73+t74+t75-t8.*t18.*2.57509e-2)+betadot.*(t16.*(-1.0./1.0e3)-t18.*2.0799e-2-t20.*3.172834e-2+t71+t72+t78+t79+t80+t81+t82-t7.*t9.*3.325e-2-t6.*t15.*2.330279e-2+t13.*t15.*9.266658999999999e-2)];
end
if nargout > 2
    t83 = betadot-q4dot+q8dot;
    omegaRightFoot = [gammadot+q5dot+q6dot-q7dot-alphadot.*t12;t8.*t83+alphadot.*t9.*t18;-t9.*t83+alphadot.*t8.*t18];
end
if nargout > 3
    IRightFoot = reshape([1.0e-4,0.0,0.0,0.0,5.0e-5,0.0,0.0,0.0,1.5e-4],[3,3]);
end
if nargout > 4
    RRightFoot = reshape([t4.*t18,t2.*t18,-t12,-t2.*t8+t4.*t9.*t12,t4.*t8+t2.*t9.*t12,t9.*t18,t2.*t9+t4.*t8.*t12,-t4.*t9+t2.*t8.*t12,t8.*t18],[3,3]);
end
