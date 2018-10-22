function vLeftAnkle = vLeftAnkle_fn(alpha,alphadot,betadot,beta,gammadot,gamma,q12,q13,q14,q15,q12dot,q13dot,q14dot,q15dot,rOxdot,rOydot,rOzdot)
%VLEFTANKLE_FN
%    VLEFTANKLE = VLEFTANKLE_FN(ALPHA,ALPHADOT,BETADOT,BETA,GAMMADOT,GAMMA,Q12,Q13,Q14,Q15,Q12DOT,Q13DOT,Q14DOT,Q15DOT,ROXDOT,ROYDOT,ROZDOT)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    18-Oct-2018 14:10:15

t2 = cos(alpha);
t3 = beta-q12;
t4 = cos(t3);
t5 = -gamma+q13+q14+q15;
t6 = cos(t5);
t7 = sin(alpha);
t8 = sin(t5);
t9 = sin(t3);
t10 = t6.*t7.*3.40719e-2;
t11 = t7.*t8.*6.88525e-3;
t12 = -gamma+q13+q14;
t13 = cos(t12);
t14 = t2.*t8.*t9.*3.40719e-2;
t15 = sin(gamma);
t16 = gamma-q13;
t17 = cos(t16);
t18 = t7.*t13.*(3.3e1./5.0e2);
t19 = cos(gamma);
t20 = sin(t12);
t21 = t2.*t9.*t20.*(3.3e1./5.0e2);
t22 = sin(beta);
t23 = t2.*t4.*t8.*6.88525e-3;
t24 = t2.*t4.*t17.*5.55e-2;
t25 = t2.*t4.*t13.*(3.3e1./5.0e2);
t26 = cos(beta);
t27 = t2.*t4.*t19.*(7.0./2.0e2);
t28 = t2.*t4.*t15.*3.225e-2;
t29 = t2.*t4.*t6.*3.40719e-2;
t30 = sin(t16);
t31 = t7.*t17.*5.55e-2;
t32 = t2.*t6.*3.40719e-2;
t33 = t2.*t8.*6.88525e-3;
t34 = t6.*t7.*t9.*6.88525e-3;
t35 = t4.*t7.*t15.*3.225e-2;
t36 = t4.*t6.*t7.*3.40719e-2;
t37 = t4.*t7.*t8.*6.88525e-3;
t38 = t4.*t7.*t17.*5.55e-2;
t39 = t4.*t7.*t13.*(3.3e1./5.0e2);
t40 = t4.*t7.*t19.*(7.0./2.0e2);
t41 = t2.*t13.*(3.3e1./5.0e2);
t42 = t2.*t17.*5.55e-2;
t43 = t7.*t9.*t30.*5.55e-2;
t44 = t4.*1.040533e-2;
t45 = t9.*t19.*(7.0./2.0e2);
t46 = t9.*t15.*3.225e-2;
t47 = t6.*t9.*3.40719e-2;
t48 = t8.*t9.*6.88525e-3;
t49 = t9.*t17.*5.55e-2;
t50 = t9.*t13.*(3.3e1./5.0e2);
t51 = t4.*t8.*3.40719e-2;
t52 = t4.*t20.*(3.3e1./5.0e2);
t53 = t4.*t6.*6.88525e-3;
t54 = t4.*t30.*5.55e-2;
vLeftAnkle = [rOxdot+alphadot.*(t2.*t6.*(-6.88525e-3)+t2.*t8.*3.40719e-2+t4.*t7.*1.040533e-2-t2.*t15.*1.2766659e-1+t2.*t19.*8.94721e-3+t2.*t20.*(3.3e1./5.0e2)-t2.*t30.*5.55e-2+t7.*t26.*3.172834e-2+t6.*t7.*t9.*3.40719e-2+t7.*t8.*t9.*6.88525e-3+t7.*t9.*t13.*(3.3e1./5.0e2)+t7.*t9.*t15.*3.225e-2+t7.*t9.*t17.*5.55e-2+t7.*t9.*t19.*(7.0./2.0e2)-t7.*t15.*t22.*2.330279e-2+t7.*t19.*t22.*9.266658999999999e-2)-gammadot.*(t10+t11+t14+t18+t21+t31+t7.*t15.*8.94721e-3+t7.*t19.*1.2766659e-1-t2.*t6.*t9.*6.88525e-3-t2.*t9.*t15.*(7.0./2.0e2)+t2.*t9.*t19.*3.225e-2-t2.*t15.*t22.*9.266658999999999e-2-t2.*t9.*t30.*5.55e-2-t2.*t19.*t22.*2.330279e-2)+q14dot.*(t10+t11+t14+t18+t21-t2.*t6.*t9.*6.88525e-3)-betadot.*(t23+t24+t25+t27+t28+t29-t2.*t9.*1.040533e-2-t2.*t22.*3.172834e-2-t2.*t15.*t26.*2.330279e-2+t2.*t19.*t26.*9.266658999999999e-2)+q15dot.*(t10+t11+t14-t2.*t6.*t9.*6.88525e-3)+q13dot.*(t10+t11+t14+t18+t21+t31-t2.*t6.*t9.*6.88525e-3-t2.*t9.*t30.*5.55e-2)+q12dot.*(t23+t24+t25+t27+t28+t29-t2.*t9.*1.040533e-2);rOydot-alphadot.*(t2.*t4.*1.040533e-2+t6.*t7.*6.88525e-3-t7.*t8.*3.40719e-2+t7.*t15.*1.2766659e-1-t7.*t19.*8.94721e-3-t7.*t20.*(3.3e1./5.0e2)+t2.*t26.*3.172834e-2+t7.*t30.*5.55e-2+t2.*t6.*t9.*3.40719e-2+t2.*t8.*t9.*6.88525e-3+t2.*t9.*t13.*(3.3e1./5.0e2)+t2.*t9.*t15.*3.225e-2+t2.*t9.*t17.*5.55e-2+t2.*t9.*t19.*(7.0./2.0e2)-t2.*t15.*t22.*2.330279e-2+t2.*t19.*t22.*9.266658999999999e-2)-q14dot.*(t32+t33+t34+t41-t7.*t8.*t9.*3.40719e-2-t7.*t9.*t20.*(3.3e1./5.0e2))-betadot.*(t35+t36+t37+t38+t39+t40-t7.*t9.*1.040533e-2-t7.*t22.*3.172834e-2-t7.*t15.*t26.*2.330279e-2+t7.*t19.*t26.*9.266658999999999e-2)+gammadot.*(t32+t33+t34+t41+t42+t43+t2.*t15.*8.94721e-3+t2.*t19.*1.2766659e-1-t7.*t8.*t9.*3.40719e-2+t7.*t9.*t15.*(7.0./2.0e2)-t7.*t9.*t19.*3.225e-2-t7.*t9.*t20.*(3.3e1./5.0e2)+t7.*t15.*t22.*9.266658999999999e-2+t7.*t19.*t22.*2.330279e-2)+q12dot.*(t35+t36+t37+t38+t39+t40-t7.*t9.*1.040533e-2)-q15dot.*(t32+t33+t34-t7.*t8.*t9.*3.40719e-2)-q13dot.*(t32+t33+t34+t41+t42+t43-t7.*t8.*t9.*3.40719e-2-t7.*t9.*t20.*(3.3e1./5.0e2));rOzdot+q15dot.*(t51-t4.*t6.*6.88525e-3)-q12dot.*(t44+t45+t46+t47+t48+t49+t50)+gammadot.*(-t51-t52+t53+t54+t4.*t15.*(7.0./2.0e2)-t4.*t19.*3.225e-2+t15.*t26.*9.266658999999999e-2+t19.*t26.*2.330279e-2)+q14dot.*(t51+t52-t4.*t6.*6.88525e-3)+q13dot.*(t51+t52-t53-t54)+betadot.*(t26.*3.172834e-2+t44+t45+t46+t47+t48+t49+t50-t15.*t22.*2.330279e-2+t19.*t22.*9.266658999999999e-2)];
