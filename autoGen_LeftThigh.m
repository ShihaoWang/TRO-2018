function [rLeftThigh,vLeftThigh,omegaLeftThigh,ILeftThigh,RLeftThigh] = autoGen_LeftThigh(rOx,rOy,rOz,alpha,beta,gamma,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15,q16,rOxdot,rOydot,rOzdot,alphadot,betadot,gammadot,q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot,q11dot,q12dot,q13dot,q14dot,q15dot,q16dot)
%AUTOGEN_LEFTTHIGH
%    [RLEFTTHIGH,VLEFTTHIGH,OMEGALEFTTHIGH,ILEFTTHIGH,RLEFTTHIGH] = AUTOGEN_LEFTTHIGH(ROX,ROY,ROZ,ALPHA,BETA,GAMMA,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13,Q14,Q15,Q16,ROXDOT,ROYDOT,ROZDOT,ALPHADOT,BETADOT,GAMMADOT,Q1DOT,Q2DOT,Q3DOT,Q4DOT,Q5DOT,Q6DOT,Q7DOT,Q8DOT,Q9DOT,Q10DOT,Q11DOT,Q12DOT,Q13DOT,Q14DOT,Q15DOT,Q16DOT)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    24-Oct-2018 20:28:47

t2 = sin(alpha);
t3 = cos(alpha);
t4 = beta-q12;
t5 = sin(gamma);
t6 = sin(t4);
t7 = gamma-q13;
t8 = cos(gamma);
t9 = sin(beta);
t10 = cos(t4);
t11 = sin(t7);
t12 = cos(beta);
t13 = cos(t7);
t14 = t3.*t10.*t13.*1.304877e-2;
t15 = t3.*t8.*t10.*(7.0./2.0e2);
t16 = t3.*t5.*t10.*3.225e-2;
t17 = t3.*t11.*1.304877e-2;
t18 = t3.*t5.*1.2766659e-1;
t19 = t2.*t5.*t9.*2.330279e-2;
t20 = t2.*t13.*1.304877e-2;
t21 = t3.*t13.*1.304877e-2;
t22 = t2.*t6.*t11.*1.304877e-2;
t23 = t2.*t8.*8.94721e-3;
t24 = t3.*t5.*t9.*2.330279e-2;
rLeftThigh = [rOx+t23+t24-t2.*t5.*1.2766659e-1-t2.*t11.*1.304877e-2-t3.*t10.*9.760599999999999e-3-t3.*t12.*3.172834e-2-t3.*t5.*t6.*3.225e-2-t3.*t6.*t8.*(7.0./2.0e2)-t3.*t8.*t9.*9.266658999999999e-2-t3.*t6.*t13.*1.304877e-2;rOy+t17+t18+t19-t3.*t8.*8.94721e-3-t2.*t10.*9.760599999999999e-3-t2.*t12.*3.172834e-2-t2.*t5.*t6.*3.225e-2-t2.*t6.*t8.*(7.0./2.0e2)-t2.*t8.*t9.*9.266658999999999e-2-t2.*t6.*t13.*1.304877e-2;rOz+t6.*9.760599999999999e-3+t9.*3.172834e-2-t5.*t10.*3.225e-2+t5.*t12.*2.330279e-2-t8.*t10.*(7.0./2.0e2)-t8.*t12.*9.266658999999999e-2-t10.*t13.*1.304877e-2];
if nargout > 1
    t25 = t2.*t5.*t10.*3.225e-2;
    t26 = t2.*t10.*t13.*1.304877e-2;
    t27 = t2.*t8.*t10.*(7.0./2.0e2);
    t28 = t10.*9.760599999999999e-3;
    t29 = t6.*t8.*(7.0./2.0e2);
    t30 = t5.*t6.*3.225e-2;
    t31 = t6.*t13.*1.304877e-2;
    vLeftThigh = [rOxdot+q12dot.*(t14+t15+t16-t3.*t6.*9.760599999999999e-3)-gammadot.*(t20+t2.*t5.*8.94721e-3+t2.*t8.*1.2766659e-1-t3.*t5.*t6.*(7.0./2.0e2)-t3.*t5.*t9.*9.266658999999999e-2+t3.*t6.*t8.*3.225e-2-t3.*t6.*t11.*1.304877e-2-t3.*t8.*t9.*2.330279e-2)+alphadot.*(-t17-t18-t19+t3.*t8.*8.94721e-3+t2.*t10.*9.760599999999999e-3+t2.*t12.*3.172834e-2+t2.*t5.*t6.*3.225e-2+t2.*t6.*t8.*(7.0./2.0e2)+t2.*t8.*t9.*9.266658999999999e-2+t2.*t6.*t13.*1.304877e-2)+q13dot.*(t20-t3.*t6.*t11.*1.304877e-2)-betadot.*(t14+t15+t16-t3.*t6.*9.760599999999999e-3-t3.*t9.*3.172834e-2-t3.*t5.*t12.*2.330279e-2+t3.*t8.*t12.*9.266658999999999e-2);rOydot-alphadot.*(-t23-t24+t2.*t5.*1.2766659e-1+t2.*t11.*1.304877e-2+t3.*t10.*9.760599999999999e-3+t3.*t12.*3.172834e-2+t3.*t5.*t6.*3.225e-2+t3.*t6.*t8.*(7.0./2.0e2)+t3.*t8.*t9.*9.266658999999999e-2+t3.*t6.*t13.*1.304877e-2)+q12dot.*(t25+t26+t27-t2.*t6.*9.760599999999999e-3)-q13dot.*(t21+t22)+gammadot.*(t21+t22+t3.*t5.*8.94721e-3+t3.*t8.*1.2766659e-1+t2.*t5.*t6.*(7.0./2.0e2)+t2.*t5.*t9.*9.266658999999999e-2-t2.*t6.*t8.*3.225e-2+t2.*t8.*t9.*2.330279e-2)-betadot.*(t25+t26+t27-t2.*t6.*9.760599999999999e-3-t2.*t9.*3.172834e-2-t2.*t5.*t12.*2.330279e-2+t2.*t8.*t12.*9.266658999999999e-2);rOzdot+betadot.*(t12.*3.172834e-2+t28+t29+t30+t31-t5.*t9.*2.330279e-2+t8.*t9.*9.266658999999999e-2)+gammadot.*(t5.*t10.*(7.0./2.0e2)+t5.*t12.*9.266658999999999e-2-t8.*t10.*3.225e-2+t8.*t12.*2.330279e-2+t10.*t11.*1.304877e-2)-q12dot.*(t28+t29+t30+t31)-q13dot.*t10.*t11.*1.304877e-2];
end
if nargout > 2
    t32 = betadot-q12dot;
    omegaLeftThigh = [gammadot-q13dot-alphadot.*t6;t13.*t32+alphadot.*t10.*t11;-t11.*t32+alphadot.*t10.*t13];
end
if nargout > 3
    ILeftThigh = reshape([1.8e-5,0.0,0.0,0.0,3.0e-4,0.0,0.0,0.0,1.8e-5],[3,3]);
end
if nargout > 4
    RLeftThigh = reshape([t3.*t10,t2.*t10,-t6,-t2.*t13+t3.*t6.*t11,t3.*t13+t2.*t6.*t11,t10.*t11,t2.*t11+t3.*t6.*t13,-t3.*t11+t2.*t6.*t13,t10.*t13],[3,3]);
end