function Main_fn()

% This is the main function of this TRO project

load('p.mat');

Robot_Config = zeros(22,1);
Robot_Velocity = ones(22,1);

Variable_Ranger([Robot_Config',Robot_Velocity']', p)

filename = ['Node_i.config'];

fid = fopen(filename,'wt');


Robot_Config_Written(Robot_Config,fid);

fclose(fid);



end

