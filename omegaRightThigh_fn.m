function omegaRightThigh = omegaRightThigh_fn(alphadot,betadot,beta,gammadot,gamma,q4,q5,q4dot,q5dot)
%OMEGARIGHTTHIGH_FN
%    OMEGARIGHTTHIGH = OMEGARIGHTTHIGH_FN(ALPHADOT,BETADOT,BETA,GAMMADOT,GAMMA,Q4,Q5,Q4DOT,Q5DOT)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    19-Oct-2018 14:57:21

t2 = gamma+q5;
t3 = beta-q4;
t4 = sin(t2);
t5 = betadot-q4dot;
t6 = cos(t2);
t7 = cos(t3);
omegaRightThigh = [gammadot+q5dot-alphadot.*sin(t3);t5.*t6+alphadot.*t4.*t7;-t4.*t5+alphadot.*t6.*t7];
