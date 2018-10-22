function rLeftHand = rLeftHand_fn(alpha,beta,gamma,q1,q2,q3,rOx,rOy,rOz)
%RLEFTHAND_FN
%    RLEFTHAND = RLEFTHAND_FN(ALPHA,BETA,GAMMA,Q1,Q2,Q3,ROX,ROY,ROZ)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    18-Oct-2018 14:10:07

t2 = sin(alpha);
t3 = pi.*(1.0./1.8e1);
t4 = cos(alpha);
t5 = gamma-q1;
t6 = beta+t3;
t7 = cos(gamma);
t8 = sin(beta);
t9 = sin(gamma);
t10 = sin(t5);
t11 = beta+q2+q3+t3;
t12 = cos(t6);
t13 = cos(t5);
t14 = beta+q2+t3;
t15 = cos(t14);
t16 = cos(t11);
t17 = cos(beta);
t18 = sin(t11);
t19 = sin(t6);
rLeftHand = [rOx+t2.*t7.*1.64721e-3+t2.*t9.*3.18887e-2+t2.*t13.*3.939380000000001e-3-t4.*t12.*1.950677e-2-t4.*t15.*(9.0./1.25e2)-t4.*t16.*4.614073e-2-t4.*t17.*6.272722e-2+t4.*t7.*t8.*3.18887e-2-t4.*t8.*t9.*1.64721e-3-t4.*t10.*t18.*2.968938e-2+t4.*t10.*t19.*2.575e-2;rOy-t4.*t7.*1.64721e-3-t4.*t9.*3.18887e-2-t2.*t12.*1.950677e-2-t2.*t15.*(9.0./1.25e2)-t4.*t13.*3.939380000000001e-3-t2.*t16.*4.614073e-2-t2.*t17.*6.272722e-2+t2.*t7.*t8.*3.18887e-2-t2.*t8.*t9.*1.64721e-3-t2.*t10.*t18.*2.968938e-2+t2.*t10.*t19.*2.575e-2;rOz+t8.*6.272722e-2+t18.*4.614073e-2+t19.*1.950677e-2+sin(t14).*(9.0./1.25e2)+t10.*t12.*2.575e-2+t7.*t17.*3.18887e-2-t9.*t17.*1.64721e-3-t10.*t16.*2.968938e-2];
