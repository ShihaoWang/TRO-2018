function omegaLeftHip = omegaLeftHip_fn(alphadot,betadot,beta,gammadot,gamma,q12,q12dot)
%OMEGALEFTHIP_FN
%    OMEGALEFTHIP = OMEGALEFTHIP_FN(ALPHADOT,BETADOT,BETA,GAMMADOT,GAMMA,Q12,Q12DOT)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    19-Oct-2018 14:57:20

t2 = beta-q12;
t3 = sin(gamma);
t4 = betadot-q12dot;
t5 = cos(gamma);
t6 = cos(t2);
omegaLeftHip = [gammadot-alphadot.*sin(t2);t4.*t5+alphadot.*t3.*t6;-t3.*t4+alphadot.*t5.*t6];