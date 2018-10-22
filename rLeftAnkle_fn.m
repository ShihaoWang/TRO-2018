function rLeftAnkle = rLeftAnkle_fn(alpha,beta,gamma,q12,q13,q14,q15,rOx,rOy,rOz)
%RLEFTANKLE_FN
%    RLEFTANKLE = RLEFTANKLE_FN(ALPHA,BETA,GAMMA,Q12,Q13,Q14,Q15,ROX,ROY,ROZ)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    18-Oct-2018 14:10:08

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
rLeftAnkle = [rOx-t2.*t6.*1.2766659e-1-t2.*t8.*6.88525e-3+t2.*t9.*3.40719e-2+t2.*t12.*8.94721e-3-t2.*t15.*5.55e-2+t2.*t16.*(3.3e1./5.0e2)-t4.*t14.*1.040533e-2-t4.*t17.*3.172834e-2-t4.*t6.*t7.*3.225e-2-t4.*t7.*t8.*3.40719e-2-t4.*t7.*t9.*6.88525e-3+t4.*t6.*t13.*2.330279e-2-t4.*t7.*t12.*(7.0./2.0e2)-t4.*t7.*t18.*5.55e-2-t4.*t12.*t13.*9.266658999999999e-2-t4.*t7.*t19.*(3.3e1./5.0e2);rOy+t4.*t6.*1.2766659e-1+t4.*t8.*6.88525e-3-t4.*t9.*3.40719e-2-t2.*t14.*1.040533e-2-t4.*t12.*8.94721e-3-t2.*t17.*3.172834e-2+t4.*t15.*5.55e-2-t4.*t16.*(3.3e1./5.0e2)-t2.*t6.*t7.*3.225e-2-t2.*t7.*t8.*3.40719e-2-t2.*t7.*t9.*6.88525e-3+t2.*t6.*t13.*2.330279e-2-t2.*t7.*t12.*(7.0./2.0e2)-t2.*t7.*t18.*5.55e-2-t2.*t12.*t13.*9.266658999999999e-2-t2.*t7.*t19.*(3.3e1./5.0e2);rOz+t7.*1.040533e-2+t13.*3.172834e-2-t6.*t14.*3.225e-2-t8.*t14.*3.40719e-2+t6.*t17.*2.330279e-2-t9.*t14.*6.88525e-3-t12.*t14.*(7.0./2.0e2)-t12.*t17.*9.266658999999999e-2-t14.*t18.*5.55e-2-t14.*t19.*(3.3e1./5.0e2)];
