function rLeftShoulder = rLeftShoulder_fn(alpha,beta,gamma,rOx,rOy,rOz)
%RLEFTSHOULDER_FN
%    RLEFTSHOULDER = RLEFTSHOULDER_FN(ALPHA,BETA,GAMMA,ROX,ROY,ROZ)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    18-Oct-2018 14:10:06

t2 = cos(alpha);
t3 = sin(alpha);
t4 = cos(gamma);
t5 = sin(beta);
t6 = sin(gamma);
t7 = pi.*(1.0./1.8e1);
t8 = beta+t7;
t9 = cos(t8);
t10 = cos(beta);
rLeftShoulder = [rOx+t3.*t4.*1.64721e-3+t3.*t6.*3.18887e-2-t2.*t9.*4.23081e-3-t2.*t10.*6.272722e-2+t2.*t4.*t5.*3.18887e-2-t2.*t5.*t6.*1.64721e-3;rOy-t2.*t4.*1.64721e-3-t2.*t6.*3.18887e-2-t3.*t9.*4.23081e-3-t3.*t10.*6.272722e-2+t3.*t4.*t5.*3.18887e-2-t3.*t5.*t6.*1.64721e-3;rOz+t5.*6.272722e-2+sin(t8).*4.23081e-3+t4.*t10.*3.18887e-2-t6.*t10.*1.64721e-3];
