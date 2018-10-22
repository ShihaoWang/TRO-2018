function omegaRightShank = omegaRightShank_fn(alphadot,betadot,beta,gammadot,gamma,q4,q5,q6,q4dot,q5dot,q6dot)
%OMEGARIGHTSHANK_FN
%    OMEGARIGHTSHANK = OMEGARIGHTSHANK_FN(ALPHADOT,BETADOT,BETA,GAMMADOT,GAMMA,Q4,Q5,Q6,Q4DOT,Q5DOT,Q6DOT)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    19-Oct-2018 14:57:21

t2 = gamma+q5+q6;
t3 = beta-q4;
t4 = sin(t2);
t5 = betadot-q4dot;
t6 = cos(t2);
t7 = cos(t3);
omegaRightShank = [gammadot+q5dot+q6dot-alphadot.*sin(t3);t5.*t6+alphadot.*t4.*t7;-t4.*t5+alphadot.*t6.*t7];