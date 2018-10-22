function Robot_Config_Written(State,fid)

rOx = State(1);     rOy = State(2);     rOz = State(3) + 0.32;     
alpha = State(4);   beta = State(5);    gamma = State(6);
q1 = State(7);      q2 = State(8);      q3 = State(9);      q4 = State(10);     q5 = State(11);     q6 = State(12);
q7 = State(13);     q8 = State(14);     q9 = State(15);     q10 = State(16);    q11 = State(17);    q12 = State(18);
q13 = State(19);    q14 = State(20);    q15 = State(21);    q16= State(22);


Row_Start = ['22\t' num2str(rOx) ' ' num2str(rOy) ' ' num2str(rOz) ' ' num2str(alpha) ' ' num2str(beta) ' ' num2str(gamma)]; 
Row_Start = [Row_Start ' ' num2str(q1) ' ' num2str(q2) ' ' num2str(q3) ' ' num2str(q4) ' ' num2str(q5) ' ' num2str(q6)]; % Start from link 6 to link 10
Row_Start = [Row_Start ' ' num2str(q7) ' ' num2str(q8) ' ' num2str(q9) ' ' num2str(q10) ' ' num2str(q11) ' ' num2str(q12)]; % Start from link 6 to link 10
Row_Start = [Row_Start ' ' num2str(q13) ' ' num2str(q14) ' ' num2str(q15) ' ' num2str(q16)]; % Start from link 6 to link 10

fprintf(fid,Row_Start);
fprintf(fid,' \n');

end

