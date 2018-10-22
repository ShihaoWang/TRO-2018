function RBody = RotBody_fn(alpha,beta,gamma)
%ROTBODY_FN
%    RBODY = ROTBODY_FN(ALPHA,BETA,GAMMA)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    18-Oct-2018 14:10:22

t2 = cos(alpha);
t3 = sin(alpha);
t4 = sin(gamma);
t5 = cos(gamma);
t6 = sin(beta);
t7 = cos(beta);
RBody = reshape([t2.*t7,t3.*t7,-t6,-t3.*t5+t2.*t4.*t6,t2.*t5+t3.*t4.*t6,t4.*t7,t3.*t4+t2.*t5.*t6,-t2.*t4+t3.*t5.*t6,t5.*t7],[3,3]);
