function [rLeftThigh,vLeftThigh,omegaLeftThigh,ILeftThigh,RLeftThigh] = autoGen_LeftThigh(rOx,rOy,rOz,alpha,beta,gamma,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,rOxdot,rOydot,rOzdot,alphadot,betadot,gammadot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot,q11dot,q12dot,q13dot,q14dot,q15dot,q16dot)
%AUTOGEN_LEFTTHIGH
%    [RLEFTTHIGH,VLEFTTHIGH,OMEGALEFTTHIGH,ILEFTTHIGH,RLEFTTHIGH] = AUTOGEN_LEFTTHIGH(ROX,ROY,ROZ,ALPHA,BETA,GAMMA,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13,Q14,Q15,Q16,ROXDOT,ROYDOT,ROZDOT,ALPHADOT,BETADOT,GAMMADOT,Q1DOT,Q2DOT,Q3DOT,Q4DOT,Q5DOT,Q6DOT,Q7DOT,Q8DOT,Q9DOT,Q10DOT,Q11DOT,Q12DOT,Q13DOT,Q14DOT,Q15DOT,Q16DOT)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    28-Nov-2018 16:30:37

t2 = sin(alpha);
t3 = sin(gamma);
t4 = t2.*t3;
t5 = cos(alpha);
t6 = cos(gamma);
t7 = sin(beta);
t8 = t5.*t6.*t7;
t9 = t4+t8;
t10 = cos(q12);
t11 = sin(q12);
t12 = cos(beta);
t13 = t3.*t5;
t17 = t2.*t6.*t7;
t14 = t13-t17;
t15 = sin(q13);
t16 = cos(q13);
t18 = t10.*t14.*(7.0./2.0e2);
t19 = t11.*t14.*9.760599999999999e-3;
t20 = t5.*t6;
t21 = t2.*t3.*t7;
t22 = t20+t21;
t23 = t10.*t14;
t24 = t2.*t11.*t12;
t25 = t23+t24;
t26 = t16.*t25.*1.304877e-2;
t27 = t3.*t5.*9.266658999999999e-2;
t28 = t2.*t11.*t12.*(7.0./2.0e2);
t29 = t2.*t6;
t31 = t3.*t5.*t7;
t30 = t29-t31;
t32 = t9.*t10;
t35 = t5.*t11.*t12;
t33 = t32-t35;
t34 = t15.*t30.*1.304877e-2;
t36 = t2.*t6.*8.94721e-3;
t37 = t5.*t11.*t12.*(7.0./2.0e2);
t38 = t7.*t11;
t39 = t6.*t10.*t12;
t40 = t38+t39;
rLeftThigh = [rOx+t34+t36+t37-t2.*t3.*9.266658999999999e-2-t5.*t12.*3.172834e-2-t9.*t10.*(7.0./2.0e2)-t9.*t11.*9.760599999999999e-3-t16.*t33.*1.304877e-2-t3.*t5.*t7.*8.94721e-3-t5.*t6.*t7.*9.266658999999999e-2-t5.*t10.*t12.*9.760599999999999e-3;rOy+t18+t19+t26+t27+t28-t5.*t6.*8.94721e-3-t2.*t12.*3.172834e-2-t15.*t22.*1.304877e-2-t2.*t3.*t7.*8.94721e-3-t2.*t6.*t7.*9.266658999999999e-2-t2.*t10.*t12.*9.760599999999999e-3;rOz+t7.*3.172834e-2-t3.*t12.*8.94721e-3+t7.*t10.*9.760599999999999e-3-t6.*t12.*9.266658999999999e-2-t7.*t11.*(7.0./2.0e2)-t16.*t40.*1.304877e-2-t6.*t10.*t12.*(7.0./2.0e2)-t6.*t11.*t12.*9.760599999999999e-3-t3.*t12.*t15.*1.304877e-2];
if nargout > 1
    t41 = t9.*t11;
    t42 = t5.*t10.*t12;
    t43 = t41+t42;
    t44 = t11.*t14;
    t56 = t2.*t10.*t12;
    t45 = t44-t56;
    t46 = betadot.*t6;
    t47 = alphadot.*t3.*t12;
    t48 = t46+t47;
    t49 = betadot.*t3;
    t55 = alphadot.*t6.*t12;
    t50 = t49-t55;
    t54 = alphadot.*t7;
    t51 = gammadot-t54;
    t52 = t7.*t10;
    t57 = t6.*t11.*t12;
    t53 = t52-t57;
    vLeftThigh = [rOxdot-gammadot.*(t2.*t3.*8.94721e-3+t2.*t6.*9.266658999999999e-2+t9.*t15.*1.304877e-2+t10.*t30.*(7.0./2.0e2)+t11.*t30.*9.760599999999999e-3-t3.*t5.*t7.*9.266658999999999e-2+t5.*t6.*t7.*8.94721e-3+t10.*t16.*t30.*1.304877e-2)+q12dot.*(t9.*t10.*(-9.760599999999999e-3)+t9.*t11.*(7.0./2.0e2)+t16.*t43.*1.304877e-2+t5.*t10.*t12.*(7.0./2.0e2)+t5.*t11.*t12.*9.760599999999999e-3)+q13dot.*(t16.*t30.*1.304877e-2+t15.*t33.*1.304877e-2)-betadot.*(t16.*(t5.*t7.*t11+t5.*t6.*t10.*t12).*1.304877e-2-t5.*t7.*3.172834e-2+t3.*t5.*t12.*8.94721e-3-t5.*t7.*t10.*9.760599999999999e-3+t5.*t6.*t12.*9.266658999999999e-2+t5.*t7.*t11.*(7.0./2.0e2)+t5.*t6.*t10.*t12.*(7.0./2.0e2)+t5.*t6.*t11.*t12.*9.760599999999999e-3+t3.*t5.*t12.*t15.*1.304877e-2)+alphadot.*(-t18-t19-t26-t27-t28+t5.*t6.*8.94721e-3+t2.*t12.*3.172834e-2+t15.*t22.*1.304877e-2+t2.*t3.*t7.*8.94721e-3+t2.*t6.*t7.*9.266658999999999e-2+t2.*t10.*t12.*9.760599999999999e-3);rOydot+gammadot.*(t3.*t5.*8.94721e-3+t5.*t6.*9.266658999999999e-2+t14.*t15.*1.304877e-2+t10.*t22.*(7.0./2.0e2)+t11.*t22.*9.760599999999999e-3+t2.*t3.*t7.*9.266658999999999e-2-t2.*t6.*t7.*8.94721e-3+t10.*t16.*t22.*1.304877e-2)+q12dot.*(t10.*t14.*9.760599999999999e-3-t11.*t14.*(7.0./2.0e2)-t16.*t45.*1.304877e-2+t2.*t10.*t12.*(7.0./2.0e2)+t2.*t11.*t12.*9.760599999999999e-3)-q13dot.*(t16.*t22.*1.304877e-2+t15.*t25.*1.304877e-2)-alphadot.*(-t34-t36-t37+t2.*t3.*9.266658999999999e-2+t5.*t12.*3.172834e-2+t9.*t10.*(7.0./2.0e2)+t9.*t11.*9.760599999999999e-3+t16.*t33.*1.304877e-2+t3.*t5.*t7.*8.94721e-3+t5.*t6.*t7.*9.266658999999999e-2+t5.*t10.*t12.*9.760599999999999e-3)-betadot.*(t16.*(t2.*t7.*t11+t2.*t6.*t10.*t12).*1.304877e-2-t2.*t7.*3.172834e-2+t2.*t3.*t12.*8.94721e-3-t2.*t7.*t10.*9.760599999999999e-3+t2.*t6.*t12.*9.266658999999999e-2+t2.*t7.*t11.*(7.0./2.0e2)+t2.*t6.*t10.*t12.*(7.0./2.0e2)+t2.*t6.*t11.*t12.*9.760599999999999e-3+t2.*t3.*t12.*t15.*1.304877e-2);rOzdot+gammadot.*(t3.*t12.*9.266658999999999e-2-t6.*t12.*8.94721e-3+t3.*t10.*t12.*(7.0./2.0e2)+t3.*t11.*t12.*9.760599999999999e-3-t6.*t12.*t15.*1.304877e-2+t3.*t10.*t12.*t16.*1.304877e-2)+betadot.*(t12.*3.172834e-2-t16.*(t11.*t12-t6.*t7.*t10).*1.304877e-2+t3.*t7.*8.94721e-3+t6.*t7.*9.266658999999999e-2+t10.*t12.*9.760599999999999e-3-t11.*t12.*(7.0./2.0e2)+t6.*t7.*t10.*(7.0./2.0e2)+t6.*t7.*t11.*9.760599999999999e-3+t3.*t7.*t15.*1.304877e-2)-q12dot.*(t7.*t10.*(7.0./2.0e2)+t7.*t11.*9.760599999999999e-3+t16.*t53.*1.304877e-2+t6.*t10.*t12.*9.760599999999999e-3-t6.*t11.*t12.*(7.0./2.0e2))+q13dot.*(t15.*t40.*1.304877e-2-t3.*t12.*t16.*1.304877e-2)];
end
if nargout > 2
    omegaLeftThigh = [q12dot.*t30-q13dot.*t43-t9.*t50-t30.*t48+t5.*t12.*t51;-q12dot.*t22+q13dot.*t45+t14.*t50+t22.*t48+t2.*t12.*t51;q13dot.*t53-t7.*t51-q12dot.*t3.*t12+t3.*t12.*t48-t6.*t12.*t50];
end
if nargout > 3
    ILeftThigh = reshape([1.8e-5,0.0,0.0,0.0,3.0e-4,0.0,0.0,0.0,1.8e-5],[3,3]);
end
if nargout > 4
    RLeftThigh = reshape([t43,-t44+t56,-t52+t57,-t16.*t30-t15.*t33,t16.*t22+t15.*t25,-t15.*t40+t3.*t12.*t16,-t15.*t30+t16.*t33,t15.*t22-t16.*t25,t16.*t40+t3.*t12.*t15],[3,3]);
end
