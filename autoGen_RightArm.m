function [rRightArm,vRightArm,omegaRightArm,IRightArm,RRightArm] = autoGen_RightArm(rOx,rOy,rOz,alpha,beta,gamma,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,rOxdot,rOydot,rOzdot,alphadot,betadot,gammadot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot,q11dot,q12dot,q13dot,q14dot,q15dot,q16dot)
%AUTOGEN_RIGHTARM
%    [RRIGHTARM,VRIGHTARM,OMEGARIGHTARM,IRIGHTARM,RRIGHTARM] = AUTOGEN_RIGHTARM(ROX,ROY,ROZ,ALPHA,BETA,GAMMA,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13,Q14,Q15,Q16,ROXDOT,ROYDOT,ROZDOT,ALPHADOT,BETADOT,GAMMADOT,Q1DOT,Q2DOT,Q3DOT,Q4DOT,Q5DOT,Q6DOT,Q7DOT,Q8DOT,Q9DOT,Q10DOT,Q11DOT,Q12DOT,Q13DOT,Q14DOT,Q15DOT,Q16DOT)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    28-Nov-2018 16:30:28

t2 = sin(alpha);
t3 = sin(gamma);
t4 = cos(alpha);
t5 = cos(gamma);
t6 = sin(beta);
t7 = cos(q9);
t8 = t2.*t5;
t33 = t3.*t4.*t6;
t9 = t8-t33;
t10 = cos(beta);
t11 = sin(q9);
t12 = t2.*t3.*9.84807753012208e-1;
t13 = t4.*t5.*t6.*9.84807753012208e-1;
t35 = t4.*t10.*1.736481776669303e-1;
t14 = t12+t13-t35;
t15 = sin(q10);
t16 = t2.*t10.*1.736481776669303e-1;
t17 = t3.*t4.*9.84807753012208e-1;
t38 = t2.*t5.*t6.*9.84807753012208e-1;
t18 = t16+t17-t38;
t19 = t4.*t5;
t20 = t2.*t3.*t6;
t21 = t19+t20;
t22 = cos(q10);
t23 = t6.*1.736481776669303e-1;
t24 = t5.*t10.*9.84807753012208e-1;
t25 = t23+t24;
t26 = t4.*t6.*1.736481776669303e-1;
t27 = t4.*t5.*t10.*9.84807753012208e-1;
t28 = t26+t27;
t29 = t2.*t3.*1.736481776669303e-1;
t30 = t4.*t10.*9.84807753012208e-1;
t31 = t4.*t5.*t6.*1.736481776669303e-1;
t32 = t29+t30+t31;
t34 = t9.*t11;
t36 = t7.*t14;
t37 = t34+t36;
t39 = t11.*t18.*4.667799999999986e-4;
t40 = t11.*t21;
t41 = t7.*t18;
t42 = t40+t41;
t43 = t15.*t42.*3.595415e-2;
t44 = t2.*t10.*9.84807753012208e-1;
t45 = t2.*t5.*t6.*1.736481776669303e-1;
t61 = t3.*t4.*1.736481776669303e-1;
t46 = t44+t45-t61;
t47 = t22.*t46.*3.595415e-2;
t48 = t2.*t10.*8.193763833222595e-2;
t49 = t2.*t5.*t6.*3.527601506266795e-2;
t50 = t2.*t3;
t51 = t4.*t5.*t6;
t52 = t50+t51;
t53 = t2.*t5.*9.84807753012208e-1;
t54 = t53-t3.*t4.*t6.*9.84807753012208e-1;
t55 = t2.*t3.*3.527601506266795e-2;
t56 = t7.*t9.*4.667799999999986e-4;
t57 = t22.*t32.*3.595415e-2;
t58 = t4.*t10.*8.193763833222595e-2;
t59 = t2.*t5.*1.64721e-3;
t60 = t4.*t5.*t6.*3.527601506266795e-2;
t62 = t2.*t6.*1.736481776669303e-1;
t63 = t2.*t5.*t10.*9.84807753012208e-1;
t64 = t62+t63;
t65 = t3.*t4;
t82 = t2.*t5.*t6;
t66 = t65-t82;
t67 = t4.*t5.*9.84807753012208e-1;
t68 = t2.*t3.*t6.*9.84807753012208e-1;
t69 = t67+t68;
t70 = t7.*t25;
t94 = t3.*t10.*t11;
t71 = t70-t94;
t72 = t6.*9.84807753012208e-1;
t86 = t5.*t10.*1.736481776669303e-1;
t73 = t72-t86;
rRightArm = [rOx+t55+t56+t57+t58+t59+t60-t11.*t14.*4.667799999999986e-4-t15.*t37.*3.595415e-2-t3.*t4.*t6.*1.64721e-3;rOy+t39+t43+t47+t48+t49-t3.*t4.*3.527601506266795e-2-t4.*t5.*1.64721e-3-t7.*t21.*4.667799999999986e-4-t2.*t3.*t6.*1.64721e-3;rOz-t6.*8.193763833222595e-2-t3.*t10.*1.64721e-3+t5.*t10.*3.527601506266795e-2-t11.*t25.*4.667799999999986e-4-t15.*t71.*3.595415e-2-t22.*t73.*3.595415e-2-t3.*t7.*t10.*4.667799999999986e-4];
if nargout > 1
    t74 = t10.*1.736481776669303e-1;
    t75 = t74-t5.*t6.*9.84807753012208e-1;
    t76 = t7.*t9;
    t92 = t11.*t14;
    t77 = t76-t92;
    t78 = t7.*t21;
    t79 = betadot.*t5;
    t80 = alphadot.*t3.*t10;
    t81 = t79+t80;
    t83 = betadot.*t3;
    t91 = alphadot.*t5.*t10;
    t84 = t83-t91;
    t87 = alphadot.*t6;
    t85 = gammadot-t87;
    t88 = t11.*t25;
    t89 = t3.*t7.*t10;
    t90 = t88+t89;
    t93 = t78-t11.*t18;
    vRightArm = [rOxdot-alphadot.*(t39+t43+t47+t48+t49-t3.*t4.*3.527601506266795e-2-t4.*t5.*1.64721e-3-t7.*t21.*4.667799999999986e-4-t2.*t3.*t6.*1.64721e-3)-gammadot.*(t22.*(t2.*t5.*1.736481776669303e-1-t3.*t4.*t6.*1.736481776669303e-1).*(-3.595415e-2)+t2.*t3.*1.64721e-3-t2.*t5.*3.527601506266795e-2+t7.*t52.*4.667799999999986e-4+t11.*t54.*4.667799999999986e-4+t15.*(t7.*t54-t11.*t52).*3.595415e-2+t3.*t4.*t6.*3.527601506266795e-2+t4.*t5.*t6.*1.64721e-3)-betadot.*(t22.*(t4.*t6.*9.84807753012208e-1-t4.*t5.*t10.*1.736481776669303e-1).*3.595415e-2+t4.*t6.*8.193763833222595e-2+t11.*t28.*4.667799999999986e-4+t15.*(t7.*t28-t3.*t4.*t10.*t11).*3.595415e-2+t3.*t4.*t10.*1.64721e-3-t4.*t5.*t10.*3.527601506266795e-2+t3.*t4.*t7.*t10.*4.667799999999986e-4)-q10dot.*(t15.*t32.*3.595415e-2+t22.*t37.*3.595415e-2)-q9dot.*(t9.*t11.*4.667799999999986e-4+t7.*t14.*4.667799999999986e-4+t15.*t77.*3.595415e-2);rOydot-gammadot.*(t22.*(t4.*t5.*1.736481776669303e-1+t2.*t3.*t6.*1.736481776669303e-1).*3.595415e-2-t3.*t4.*1.64721e-3+t4.*t5.*3.527601506266795e-2-t7.*t66.*4.667799999999986e-4-t11.*t69.*4.667799999999986e-4-t15.*(t7.*t69-t11.*t66).*3.595415e-2+t2.*t3.*t6.*3.527601506266795e-2+t2.*t5.*t6.*1.64721e-3)+alphadot.*(t55+t56+t57+t58+t59+t60-t11.*t14.*4.667799999999986e-4-t15.*t37.*3.595415e-2-t3.*t4.*t6.*1.64721e-3)-betadot.*(t22.*(t2.*t6.*9.84807753012208e-1-t2.*t5.*t10.*1.736481776669303e-1).*3.595415e-2+t2.*t6.*8.193763833222595e-2+t11.*t64.*4.667799999999986e-4+t15.*(t7.*t64-t2.*t3.*t10.*t11).*3.595415e-2+t2.*t3.*t10.*1.64721e-3-t2.*t5.*t10.*3.527601506266795e-2+t2.*t3.*t7.*t10.*4.667799999999986e-4)-q10dot.*(t15.*t46.*3.595415e-2-t22.*t42.*3.595415e-2)+q9dot.*(t7.*t18.*4.667799999999986e-4+t11.*t21.*4.667799999999986e-4+t15.*t93.*3.595415e-2);rOzdot+q9dot.*(t7.*t25.*(-4.667799999999986e-4)+t15.*t90.*3.595415e-2+t3.*t10.*t11.*4.667799999999986e-4)-betadot.*(t10.*8.193763833222595e-2+t15.*(t7.*t75+t3.*t6.*t11).*3.595415e-2-t3.*t6.*1.64721e-3+t5.*t6.*3.527601506266795e-2+t11.*t75.*4.667799999999986e-4+t22.*(t10.*9.84807753012208e-1+t5.*t6.*1.736481776669303e-1).*3.595415e-2-t3.*t6.*t7.*4.667799999999986e-4)-gammadot.*(t15.*(t3.*t7.*t10.*9.84807753012208e-1+t5.*t10.*t11).*(-3.595415e-2)+t3.*t10.*3.527601506266795e-2+t5.*t10.*1.64721e-3+t5.*t7.*t10.*4.667799999999986e-4-t3.*t10.*t11.*4.59688562951037e-4+t3.*t10.*t22.*6.243372627063463e-3)+q10dot.*(t15.*t73.*3.595415e-2-t22.*t71.*3.595415e-2)];
end
if nargout > 2
    omegaRightArm = [-q10dot.*t77+q9dot.*t32-t9.*t81-t52.*t84+t4.*t10.*t85;q10dot.*t93+q9dot.*t46+t21.*t81+t66.*t84+t2.*t10.*t85;q10dot.*t90-q9dot.*t73-t6.*t85+t3.*t10.*t81-t5.*t10.*t84];
end
if nargout > 3
    IRightArm = reshape([3.515445e-5,0.0,0.0,0.0,1.25e-4,0.0,0.0,0.0,1.25e-4],[3,3]);
end
if nargout > 4
    RRightArm = reshape([-t15.*t37+t22.*t32,t15.*t42+t22.*t46,-t15.*t71-t22.*t73,-t76+t92,t93,t90,t15.*t32+t22.*t37,t15.*t46-t22.*t42,-t15.*t73+t22.*t71],[3,3]);
end
