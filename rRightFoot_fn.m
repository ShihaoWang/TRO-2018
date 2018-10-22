function rRightFoot = rRightFoot_fn(alpha,beta,gamma,q4,q5,q6,q7,q8,rOx,rOy,rOz)
%RRIGHTFOOT_FN
%    RRIGHTFOOT = RRIGHTFOOT_FN(ALPHA,BETA,GAMMA,Q4,Q5,Q6,Q7,Q8,ROX,ROY,ROZ)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    18-Oct-2018 14:10:10

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
rRightFoot = [rOx-t2.*t6.*1.2766659e-1-t2.*t8.*7.4991e-3-t2.*t9.*5.843266e-2+t2.*t13.*8.94721e-3-t2.*t17.*(3.3e1./5.0e2)+t4.*t16.*(1.0./1.0e3)-t2.*t19.*5.55e-2+t4.*t18.*2.0799e-2+t4.*t20.*3.172834e-2-t4.*t6.*t7.*3.225e-2-t4.*t7.*t8.*(7.0./2.0e2)+t4.*t7.*t9.*3.325e-2-t4.*t7.*t13.*(7.0./2.0e2)-t4.*t8.*t12.*2.343266e-2+t4.*t6.*t15.*2.330279e-2-t4.*t9.*t12.*2.57509e-2-t4.*t7.*t21.*(3.3e1./5.0e2)-t4.*t13.*t15.*9.266658999999999e-2-t4.*t7.*t22.*5.55e-2;rOy+t4.*t6.*1.2766659e-1+t4.*t8.*7.4991e-3+t4.*t9.*5.843266e-2-t4.*t13.*8.94721e-3+t2.*t16.*(1.0./1.0e3)+t2.*t18.*2.0799e-2+t4.*t17.*(3.3e1./5.0e2)+t2.*t20.*3.172834e-2+t4.*t19.*5.55e-2-t2.*t6.*t7.*3.225e-2-t2.*t7.*t8.*(7.0./2.0e2)+t2.*t7.*t9.*3.325e-2-t2.*t7.*t13.*(7.0./2.0e2)-t2.*t8.*t12.*2.343266e-2+t2.*t6.*t15.*2.330279e-2-t2.*t9.*t12.*2.57509e-2-t2.*t7.*t21.*(3.3e1./5.0e2)-t2.*t13.*t15.*9.266658999999999e-2-t2.*t7.*t22.*5.55e-2;rOz-t7.*(1.0./1.0e3)-t12.*2.0799e-2-t15.*3.172834e-2-t6.*t16.*3.225e-2-t8.*t16.*(7.0./2.0e2)+t9.*t16.*3.325e-2+t6.*t20.*2.330279e-2-t8.*t18.*2.343266e-2-t9.*t18.*2.57509e-2-t13.*t16.*(7.0./2.0e2)-t13.*t20.*9.266658999999999e-2-t16.*t21.*(3.3e1./5.0e2)-t16.*t22.*5.55e-2];
