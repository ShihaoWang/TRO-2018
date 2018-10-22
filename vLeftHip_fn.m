function vLeftHip = vLeftHip_fn(alpha,alphadot,betadot,beta,gammadot,gamma,q12,q12dot,rOxdot,rOydot,rOzdot)
%VLEFTHIP_FN
%    VLEFTHIP = VLEFTHIP_FN(ALPHA,ALPHADOT,BETADOT,BETA,GAMMADOT,GAMMA,Q12,Q12DOT,ROXDOT,ROYDOT,ROZDOT)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    18-Oct-2018 14:10:13

t2 = sin(alpha);
t3 = sin(gamma);
t4 = cos(alpha);
t5 = cos(gamma);
t6 = sin(beta);
t7 = beta-q12;
t8 = sin(t7);
t9 = cos(t7);
t10 = cos(beta);
t11 = t4.*t5.*t9.*2.68635e-3;
t12 = t3.*t4.*t9.*1.458538e-2;
t13 = t2.*t8.*1.065566e-2;
t14 = t9.*1.065566e-2;
t15 = t5.*t8.*2.68635e-3;
t16 = t3.*t8.*1.458538e-2;
vLeftHip = [rOxdot+alphadot.*(t3.*t4.*(-9.535293999999999e-2)-t4.*t5.*8.71741e-3+t2.*t9.*1.065566e-2+t2.*t10.*3.172834e-2-t2.*t3.*t6.*2.330279e-2+t2.*t3.*t8.*1.458538e-2+t2.*t5.*t6.*9.266658999999999e-2+t2.*t5.*t8.*2.68635e-3)-betadot.*(t11+t12-t4.*t6.*3.172834e-2-t4.*t8.*1.065566e-2-t3.*t4.*t10.*2.330279e-2+t4.*t5.*t10.*9.266658999999999e-2)+q12dot.*(t11+t12-t4.*t8.*1.065566e-2)+gammadot.*(t2.*t3.*8.71741e-3-t2.*t5.*9.535293999999999e-2+t3.*t4.*t6.*9.266658999999999e-2+t3.*t4.*t8.*2.68635e-3+t4.*t5.*t6.*2.330279e-2-t4.*t5.*t8.*1.458538e-2);rOydot-alphadot.*(t2.*t3.*9.535293999999999e-2+t2.*t5.*8.71741e-3+t4.*t9.*1.065566e-2+t4.*t10.*3.172834e-2-t3.*t4.*t6.*2.330279e-2+t3.*t4.*t8.*1.458538e-2+t4.*t5.*t6.*9.266658999999999e-2+t4.*t5.*t8.*2.68635e-3)+q12dot.*(-t13+t2.*t3.*t9.*1.458538e-2+t2.*t5.*t9.*2.68635e-3)+betadot.*(t13+t2.*t6.*3.172834e-2-t2.*t3.*t9.*1.458538e-2+t2.*t3.*t10.*2.330279e-2-t2.*t5.*t9.*2.68635e-3-t2.*t5.*t10.*9.266658999999999e-2)+gammadot.*(t3.*t4.*(-8.71741e-3)+t4.*t5.*9.535293999999999e-2+t2.*t3.*t6.*9.266658999999999e-2+t2.*t3.*t8.*2.68635e-3+t2.*t5.*t6.*2.330279e-2-t2.*t5.*t8.*1.458538e-2);rOzdot-q12dot.*(t14+t15+t16)+gammadot.*(t3.*t9.*2.68635e-3+t3.*t10.*9.266658999999999e-2-t5.*t9.*1.458538e-2+t5.*t10.*2.330279e-2)+betadot.*(t10.*3.172834e-2+t14+t15+t16-t3.*t6.*2.330279e-2+t5.*t6.*9.266658999999999e-2)];