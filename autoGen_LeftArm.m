function [rLeftArm,vLeftArm,omegaLeftArm,ILeftArm,RLeftArm] = autoGen_LeftArm(rOx,rOy,rOz,alpha,beta,gamma,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,rOxdot,rOydot,rOzdot,alphadot,betadot,gammadot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot,q11dot,q12dot,q13dot,q14dot,q15dot,q16dot)
%AUTOGEN_LEFTARM
%    [RLEFTARM,VLEFTARM,OMEGALEFTARM,ILEFTARM,RLEFTARM] = AUTOGEN_LEFTARM(ROX,ROY,ROZ,ALPHA,BETA,GAMMA,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13,Q14,Q15,Q16,ROXDOT,ROYDOT,ROZDOT,ALPHADOT,BETADOT,GAMMADOT,Q1DOT,Q2DOT,Q3DOT,Q4DOT,Q5DOT,Q6DOT,Q7DOT,Q8DOT,Q9DOT,Q10DOT,Q11DOT,Q12DOT,Q13DOT,Q14DOT,Q15DOT,Q16DOT)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    28-Nov-2018 16:30:19

t2 = sin(alpha);
t3 = sin(gamma);
t4 = cos(alpha);
t5 = cos(gamma);
t6 = sin(beta);
t7 = cos(q1);
t8 = t2.*t5;
t31 = t3.*t4.*t6;
t9 = t8-t31;
t10 = cos(beta);
t11 = sin(q1);
t12 = t2.*t3.*9.84807753012208e-1;
t13 = t4.*t10.*1.736481776669303e-1;
t14 = t4.*t5.*t6.*9.84807753012208e-1;
t15 = t12+t13+t14;
t16 = sin(q2);
t17 = t2.*t10.*1.736481776669303e-1;
t18 = t2.*t5.*t6.*9.84807753012208e-1;
t34 = t3.*t4.*9.84807753012208e-1;
t19 = t17+t18-t34;
t20 = t4.*t5;
t21 = t2.*t3.*t6;
t22 = t20+t21;
t23 = cos(q2);
t24 = t6.*1.736481776669303e-1;
t65 = t5.*t10.*9.84807753012208e-1;
t25 = t24-t65;
t26 = t4.*t6.*1.736481776669303e-1;
t27 = t26-t4.*t5.*t10.*9.84807753012208e-1;
t28 = t2.*t3.*1.736481776669303e-1;
t29 = t4.*t5.*t6.*1.736481776669303e-1;
t52 = t4.*t10.*9.84807753012208e-1;
t30 = t28+t29-t52;
t32 = t9.*t11;
t49 = t7.*t15;
t33 = t32-t49;
t35 = t11.*t19.*4.667799999999986e-4;
t36 = t11.*t22;
t37 = t7.*t19;
t38 = t36+t37;
t39 = t16.*t38.*3.595415e-2;
t40 = t2.*t10.*9.84807753012208e-1;
t41 = t3.*t4.*1.736481776669303e-1;
t57 = t2.*t5.*t6.*1.736481776669303e-1;
t42 = t40+t41-t57;
t43 = t2.*t5.*t6.*3.527601506266795e-2;
t44 = t2.*t3;
t45 = t4.*t5.*t6;
t46 = t44+t45;
t47 = t2.*t5.*9.84807753012208e-1;
t48 = t47-t3.*t4.*t6.*9.84807753012208e-1;
t50 = t2.*t3.*3.527601506266795e-2;
t51 = t7.*t9.*4.667799999999986e-4;
t53 = t23.*t30.*3.595415e-2;
t54 = t11.*t15.*4.667799999999986e-4;
t55 = t2.*t5.*1.64721e-3;
t56 = t4.*t5.*t6.*3.527601506266795e-2;
t58 = t2.*t6.*1.736481776669303e-1;
t59 = t58-t2.*t5.*t10.*9.84807753012208e-1;
t60 = t3.*t4;
t81 = t2.*t5.*t6;
t61 = t60-t81;
t62 = t4.*t5.*9.84807753012208e-1;
t63 = t2.*t3.*t6.*9.84807753012208e-1;
t64 = t62+t63;
t66 = t7.*t25;
t91 = t3.*t10.*t11;
t67 = t66-t91;
t68 = t6.*9.84807753012208e-1;
t69 = t5.*t10.*1.736481776669303e-1;
t70 = t68+t69;
rLeftArm = [rOx+t50+t51+t53+t54+t55+t56-t4.*t10.*8.193763833222595e-2-t16.*t33.*3.595415e-2-t3.*t4.*t6.*1.64721e-3;rOy+t35+t39+t43-t3.*t4.*3.527601506266795e-2-t4.*t5.*1.64721e-3-t2.*t10.*8.193763833222595e-2-t7.*t22.*4.667799999999986e-4-t23.*t42.*3.595415e-2-t2.*t3.*t6.*1.64721e-3;rOz+t6.*8.193763833222595e-2-t3.*t10.*1.64721e-3+t5.*t10.*3.527601506266795e-2-t11.*t25.*4.667799999999986e-4-t16.*t67.*3.595415e-2+t23.*t70.*3.595415e-2-t3.*t7.*t10.*4.667799999999986e-4];
if nargout > 1
    t71 = t10.*1.736481776669303e-1;
    t72 = t5.*t6.*9.84807753012208e-1;
    t73 = t71+t72;
    t74 = t7.*t9;
    t75 = t11.*t15;
    t76 = t74+t75;
    t77 = t7.*t22;
    t78 = betadot.*t5;
    t79 = alphadot.*t3.*t10;
    t80 = t78+t79;
    t82 = betadot.*t3;
    t89 = alphadot.*t5.*t10;
    t83 = t82-t89;
    t85 = alphadot.*t6;
    t84 = gammadot-t85;
    t86 = t11.*t25;
    t87 = t3.*t7.*t10;
    t88 = t86+t87;
    t90 = t77-t11.*t19;
    vLeftArm = [rOxdot+gammadot.*(t23.*(t2.*t5.*1.736481776669303e-1-t3.*t4.*t6.*1.736481776669303e-1).*3.595415e-2-t2.*t3.*1.64721e-3+t2.*t5.*3.527601506266795e-2-t7.*t46.*4.667799999999986e-4+t11.*t48.*4.667799999999986e-4+t16.*(t7.*t48+t11.*t46).*3.595415e-2-t3.*t4.*t6.*3.527601506266795e-2-t4.*t5.*t6.*1.64721e-3)-betadot.*(t23.*(t4.*t6.*9.84807753012208e-1+t4.*t5.*t10.*1.736481776669303e-1).*(-3.595415e-2)-t4.*t6.*8.193763833222595e-2+t11.*t27.*4.667799999999986e-4+t16.*(t7.*t27-t3.*t4.*t10.*t11).*3.595415e-2+t3.*t4.*t10.*1.64721e-3-t4.*t5.*t10.*3.527601506266795e-2+t3.*t4.*t7.*t10.*4.667799999999986e-4)-q2dot.*(t16.*t30.*3.595415e-2+t23.*t33.*3.595415e-2)+alphadot.*(-t35-t39-t43+t3.*t4.*3.527601506266795e-2+t4.*t5.*1.64721e-3+t2.*t10.*8.193763833222595e-2+t7.*t22.*4.667799999999986e-4+t23.*t42.*3.595415e-2+t2.*t3.*t6.*1.64721e-3)-q1dot.*(t9.*t11.*4.667799999999986e-4-t7.*t15.*4.667799999999986e-4+t16.*t76.*3.595415e-2);rOydot+alphadot.*(t50+t51+t53+t54+t55+t56-t4.*t10.*8.193763833222595e-2-t16.*t33.*3.595415e-2-t3.*t4.*t6.*1.64721e-3)-gammadot.*(t23.*(t4.*t5.*1.736481776669303e-1+t2.*t3.*t6.*1.736481776669303e-1).*3.595415e-2-t3.*t4.*1.64721e-3+t4.*t5.*3.527601506266795e-2-t7.*t61.*4.667799999999986e-4+t11.*t64.*4.667799999999986e-4+t16.*(t7.*t64+t11.*t61).*3.595415e-2+t2.*t3.*t6.*3.527601506266795e-2+t2.*t5.*t6.*1.64721e-3)-betadot.*(t23.*(t2.*t6.*9.84807753012208e-1+t2.*t5.*t10.*1.736481776669303e-1).*(-3.595415e-2)-t2.*t6.*8.193763833222595e-2+t11.*t59.*4.667799999999986e-4+t16.*(t7.*t59-t2.*t3.*t10.*t11).*3.595415e-2+t2.*t3.*t10.*1.64721e-3-t2.*t5.*t10.*3.527601506266795e-2+t2.*t3.*t7.*t10.*4.667799999999986e-4)+q2dot.*(t16.*t42.*3.595415e-2+t23.*t38.*3.595415e-2)+q1dot.*(t7.*t19.*4.667799999999986e-4+t11.*t22.*4.667799999999986e-4+t16.*t90.*3.595415e-2);rOzdot+q1dot.*(t7.*t25.*(-4.667799999999986e-4)+t16.*t88.*3.595415e-2+t3.*t10.*t11.*4.667799999999986e-4)+betadot.*(t10.*8.193763833222595e-2-t16.*(t7.*t73+t3.*t6.*t11).*3.595415e-2+t3.*t6.*1.64721e-3-t5.*t6.*3.527601506266795e-2-t11.*t73.*4.667799999999986e-4+t23.*(t10.*9.84807753012208e-1-t5.*t6.*1.736481776669303e-1).*3.595415e-2+t3.*t6.*t7.*4.667799999999986e-4)-gammadot.*(t16.*(t3.*t7.*t10.*9.84807753012208e-1-t5.*t10.*t11).*3.595415e-2+t3.*t10.*3.527601506266795e-2+t5.*t10.*1.64721e-3+t5.*t7.*t10.*4.667799999999986e-4+t3.*t10.*t11.*4.59688562951037e-4+t3.*t10.*t23.*6.243372627063463e-3)-q2dot.*(t16.*t70.*3.595415e-2+t23.*t67.*3.595415e-2)];
end
if nargout > 2
    omegaLeftArm = [q1dot.*t30-q2dot.*t76-t9.*t80-t46.*t83+t4.*t10.*t84;-q1dot.*t42+q2dot.*t90+t22.*t80+t61.*t83+t2.*t10.*t84;q1dot.*t70+q2dot.*t88-t6.*t84+t3.*t10.*t80-t5.*t10.*t83];
end
if nargout > 3
    ILeftArm = reshape([3.515445e-5,0.0,0.0,0.0,1.25e-4,0.0,0.0,0.0,1.25e-4],[3,3]);
end
if nargout > 4
    RLeftArm = reshape([t16.*t33-t23.*t30,-t16.*t38+t23.*t42,t16.*t67-t23.*t70,-t74-t75,t90,t88,-t16.*t30-t23.*t33,t16.*t42+t23.*t38,-t16.*t70-t23.*t67],[3,3]);
end
