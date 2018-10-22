function vRightShoulder = vRightShoulder_fn(alpha,alphadot,betadot,beta,gammadot,gamma,rOxdot,rOydot,rOzdot)
%VRIGHTSHOULDER_FN
%    VRIGHTSHOULDER = VRIGHTSHOULDER_FN(ALPHA,ALPHADOT,BETADOT,BETA,GAMMADOT,GAMMA,ROXDOT,ROYDOT,ROZDOT)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    18-Oct-2018 14:10:12

t2 = sin(alpha);
t3 = cos(alpha);
t4 = cos(gamma);
t5 = sin(beta);
t6 = sin(gamma);
t9 = pi.*(1.0./1.8e1);
t7 = beta-t9;
t8 = cos(beta);
t10 = cos(t7);
t11 = sin(t7);
vRightShoulder = [rOxdot+alphadot.*(t3.*t4.*1.64721e-3+t3.*t6.*3.18887e-2-t2.*t8.*6.272722e-2-t2.*t10.*4.23081e-3-t2.*t4.*t5.*3.18887e-2+t2.*t5.*t6.*1.64721e-3)-betadot.*(t3.*t5.*6.272722e-2+t3.*t11.*4.23081e-3-t3.*t4.*t8.*3.18887e-2+t3.*t6.*t8.*1.64721e-3)-gammadot.*(t2.*t4.*(-3.18887e-2)+t2.*t6.*1.64721e-3+t3.*t4.*t5.*1.64721e-3+t3.*t5.*t6.*3.18887e-2);rOydot+alphadot.*(t2.*t4.*1.64721e-3+t2.*t6.*3.18887e-2+t3.*t8.*6.272722e-2+t3.*t10.*4.23081e-3+t3.*t4.*t5.*3.18887e-2-t3.*t5.*t6.*1.64721e-3)-betadot.*(t2.*t5.*6.272722e-2+t2.*t11.*4.23081e-3-t2.*t4.*t8.*3.18887e-2+t2.*t6.*t8.*1.64721e-3)-gammadot.*(t3.*t4.*3.18887e-2-t3.*t6.*1.64721e-3+t2.*t4.*t5.*1.64721e-3+t2.*t5.*t6.*3.18887e-2);rOzdot-gammadot.*(t4.*t8.*1.64721e-3+t6.*t8.*3.18887e-2)-betadot.*(t8.*6.272722e-2+t10.*4.23081e-3+t4.*t5.*3.18887e-2-t5.*t6.*1.64721e-3)];