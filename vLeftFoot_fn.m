function vLeftFoot = vLeftFoot_fn(alpha,alphadot,betadot,beta,gammadot,gamma,q12,q13,q14,q15,q16,q12dot,q13dot,q14dot,q15dot,q16dot,rOxdot,rOydot,rOzdot)
%VLEFTFOOT_FN
%    VLEFTFOOT = VLEFTFOOT_FN(ALPHA,ALPHADOT,BETADOT,BETA,GAMMADOT,GAMMA,Q12,Q13,Q14,Q15,Q16,Q12DOT,Q13DOT,Q14DOT,Q15DOT,Q16DOT,ROXDOT,ROYDOT,ROZDOT)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    18-Oct-2018 14:10:17

t2 = cos(alpha);
t3 = -gamma+q13+q14+q15;
t4 = sin(alpha);
t5 = beta-q12;
t6 = cos(gamma);
t7 = sin(t5);
t8 = sin(gamma);
t9 = cos(t3);
t10 = sin(t3);
t11 = gamma-q13;
t12 = beta-q12+q16;
t13 = sin(t12);
t14 = -gamma+q13+q14;
t15 = sin(beta);
t16 = cos(t14);
t17 = sin(t14);
t18 = cos(t12);
t19 = t4.*t9.*5.843266e-2;
t20 = t4.*t10.*7.4991e-3;
t21 = cos(t11);
t22 = t4.*t16.*(3.3e1./5.0e2);
t23 = t2.*t7.*t10.*(7.0./2.0e2);
t24 = sin(t11);
t25 = t2.*t9.*t13.*2.57509e-2;
t26 = t2.*t10.*t13.*2.343266e-2;
t27 = t2.*t7.*t17.*(3.3e1./5.0e2);
t28 = t2.*t13.*2.0799e-2;
t29 = cos(t5);
t30 = t2.*t10.*t18.*2.57509e-2;
t31 = cos(beta);
t32 = t4.*t21.*5.55e-2;
t33 = t2.*t10.*t29.*3.325e-2;
t34 = t2.*t21.*t29.*5.55e-2;
t35 = t2.*t9.*t18.*2.343266e-2;
t36 = t2.*t16.*t29.*(3.3e1./5.0e2);
t37 = t2.*t6.*t29.*(7.0./2.0e2);
t38 = t2.*t8.*t29.*3.225e-2;
t39 = t2.*t9.*t29.*(7.0./2.0e2);
t40 = t2.*t9.*5.843266e-2;
t41 = t2.*t10.*7.4991e-3;
t42 = t4.*t7.*t9.*3.325e-2;
t43 = t4.*t9.*t18.*2.343266e-2;
t44 = t4.*t13.*2.0799e-2;
t45 = t4.*t8.*t29.*3.225e-2;
t46 = t4.*t9.*t29.*(7.0./2.0e2);
t47 = t4.*t10.*t29.*3.325e-2;
t48 = t4.*t21.*t29.*5.55e-2;
t49 = t4.*t10.*t18.*2.57509e-2;
t50 = t4.*t16.*t29.*(3.3e1./5.0e2);
t51 = t4.*t6.*t29.*(7.0./2.0e2);
t52 = t2.*t16.*(3.3e1./5.0e2);
t53 = t2.*t21.*5.55e-2;
t54 = t4.*t7.*t24.*5.55e-2;
t55 = t10.*t29.*(7.0./2.0e2);
t56 = t9.*t18.*2.57509e-2;
t57 = t10.*t18.*2.343266e-2;
t58 = t17.*t29.*(3.3e1./5.0e2);
t59 = t18.*2.0799e-2;
t60 = t9.*t13.*2.343266e-2;
t61 = t29.*(1.0./1.0e3);
t62 = t6.*t7.*(7.0./2.0e2);
t63 = t7.*t8.*3.225e-2;
t64 = t7.*t9.*(7.0./2.0e2);
t65 = t7.*t10.*3.325e-2;
t66 = t7.*t21.*5.55e-2;
t67 = t7.*t16.*(3.3e1./5.0e2);
vLeftFoot = [rOxdot+q14dot.*(t19+t20+t22+t23+t25+t26+t27-t2.*t7.*t9.*3.325e-2)-betadot.*(-t28-t30+t33+t34+t35+t36+t37+t38+t39-t2.*t7.*(1.0./1.0e3)-t2.*t15.*3.172834e-2+t2.*t6.*t31.*9.266658999999999e-2-t2.*t8.*t31.*2.330279e-2)+q15dot.*(t19+t20+t23+t25+t26-t2.*t7.*t9.*3.325e-2)+q13dot.*(t19+t20+t22+t23+t25+t26+t27+t32-t2.*t7.*t9.*3.325e-2-t2.*t7.*t24.*5.55e-2)+q12dot.*(-t28-t30+t33+t34+t35+t36+t37+t38+t39-t2.*t7.*(1.0./1.0e3))+alphadot.*(t2.*t6.*8.94721e-3-t2.*t8.*1.2766659e-1-t2.*t9.*7.4991e-3+t2.*t10.*5.843266e-2+t2.*t17.*(3.3e1./5.0e2)+t4.*t18.*2.0799e-2-t2.*t24.*5.55e-2+t4.*t29.*(1.0./1.0e3)+t4.*t31.*3.172834e-2+t4.*t6.*t7.*(7.0./2.0e2)+t4.*t7.*t8.*3.225e-2+t4.*t7.*t9.*(7.0./2.0e2)+t4.*t7.*t10.*3.325e-2+t4.*t6.*t15.*9.266658999999999e-2+t4.*t9.*t13.*2.343266e-2+t4.*t7.*t16.*(3.3e1./5.0e2)-t4.*t8.*t15.*2.330279e-2-t4.*t10.*t13.*2.57509e-2+t4.*t7.*t21.*5.55e-2)+q16dot.*(t28+t30-t2.*t9.*t18.*2.343266e-2)-gammadot.*(t19+t20+t22+t23+t25+t26+t27+t32+t4.*t6.*1.2766659e-1+t4.*t8.*8.94721e-3+t2.*t6.*t7.*3.225e-2-t2.*t7.*t8.*(7.0./2.0e2)-t2.*t7.*t9.*3.325e-2-t2.*t6.*t15.*2.330279e-2-t2.*t8.*t15.*9.266658999999999e-2-t2.*t7.*t24.*5.55e-2);rOydot-q15dot.*(t40+t41+t42-t4.*t7.*t10.*(7.0./2.0e2)-t4.*t9.*t13.*2.57509e-2-t4.*t10.*t13.*2.343266e-2)-q13dot.*(t40+t41+t42+t52+t53+t54-t4.*t7.*t10.*(7.0./2.0e2)-t4.*t9.*t13.*2.57509e-2-t4.*t10.*t13.*2.343266e-2-t4.*t7.*t17.*(3.3e1./5.0e2))+q12dot.*(t43-t44+t45+t46+t47+t48-t49+t50+t51-t4.*t7.*(1.0./1.0e3))-alphadot.*(t4.*t6.*(-8.94721e-3)+t4.*t8.*1.2766659e-1+t4.*t9.*7.4991e-3-t4.*t10.*5.843266e-2+t2.*t18.*2.0799e-2-t4.*t17.*(3.3e1./5.0e2)+t4.*t24.*5.55e-2+t2.*t29.*(1.0./1.0e3)+t2.*t31.*3.172834e-2+t2.*t6.*t7.*(7.0./2.0e2)+t2.*t7.*t8.*3.225e-2+t2.*t7.*t9.*(7.0./2.0e2)+t2.*t7.*t10.*3.325e-2+t2.*t6.*t15.*9.266658999999999e-2+t2.*t9.*t13.*2.343266e-2+t2.*t7.*t16.*(3.3e1./5.0e2)-t2.*t8.*t15.*2.330279e-2-t2.*t10.*t13.*2.57509e-2+t2.*t7.*t21.*5.55e-2)-q14dot.*(t40+t41+t42+t52-t4.*t7.*t10.*(7.0./2.0e2)-t4.*t9.*t13.*2.57509e-2-t4.*t10.*t13.*2.343266e-2-t4.*t7.*t17.*(3.3e1./5.0e2))-betadot.*(t43+t45+t46+t47+t48+t50+t51-t4.*t7.*(1.0./1.0e3)-t4.*t13.*2.0799e-2-t4.*t15.*3.172834e-2-t4.*t10.*t18.*2.57509e-2+t4.*t6.*t31.*9.266658999999999e-2-t4.*t8.*t31.*2.330279e-2)+gammadot.*(t40+t41+t42+t52+t53+t54+t2.*t6.*1.2766659e-1+t2.*t8.*8.94721e-3-t4.*t6.*t7.*3.225e-2+t4.*t7.*t8.*(7.0./2.0e2)-t4.*t7.*t10.*(7.0./2.0e2)+t4.*t6.*t15.*2.330279e-2-t4.*t9.*t13.*2.57509e-2+t4.*t8.*t15.*9.266658999999999e-2-t4.*t10.*t13.*2.343266e-2-t4.*t7.*t17.*(3.3e1./5.0e2))+q16dot.*(-t43+t44+t49);rOzdot+q16dot.*(t59+t60-t10.*t13.*2.57509e-2)+q14dot.*(t55+t56+t57+t58-t9.*t29.*3.325e-2)-gammadot.*(t55+t56+t57+t58+t6.*t29.*3.225e-2-t6.*t31.*2.330279e-2-t8.*t29.*(7.0./2.0e2)-t9.*t29.*3.325e-2-t8.*t31.*9.266658999999999e-2-t24.*t29.*5.55e-2)-q12dot.*(t59+t60+t61+t62+t63+t64+t65+t66+t67-t10.*t13.*2.57509e-2)+q13dot.*(t55+t56+t57+t58-t9.*t29.*3.325e-2-t24.*t29.*5.55e-2)+betadot.*(t31.*3.172834e-2+t59+t60+t61+t62+t63+t64+t65+t66+t67+t6.*t15.*9.266658999999999e-2-t8.*t15.*2.330279e-2-t10.*t13.*2.57509e-2)+q15dot.*(t55+t56+t57-t9.*t29.*3.325e-2)];
