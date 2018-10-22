function omegaRightHand = omegaRightHand_fn(alphadot,betadot,beta,gammadot,gamma,q9,q10,q11,q10dot,q11dot,q9dot)
%OMEGARIGHTHAND_FN
%    OMEGARIGHTHAND = OMEGARIGHTHAND_FN(ALPHADOT,BETADOT,BETA,GAMMADOT,GAMMA,Q9,Q10,Q11,Q10DOT,Q11DOT,Q9DOT)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    19-Oct-2018 14:57:20

t2 = gamma+q9;
t7 = pi.*(1.0./1.8e1);
t3 = beta+q10+q11-t7;
t4 = sin(t2);
t5 = betadot+q10dot+q11dot;
t6 = cos(t2);
t8 = cos(t3);
omegaRightHand = [gammadot+q9dot-alphadot.*sin(t3);t5.*t6+alphadot.*t4.*t8;-t4.*t5+alphadot.*t6.*t8];
