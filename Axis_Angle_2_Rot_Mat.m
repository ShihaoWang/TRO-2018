function Rot = Axis_Angle_2_Rot_Mat(n, theta)

% This function will compute the rotation matrix based on the rotation axis n and the rotation angle theta
[row,col] = size(n);
if (col>row)
    n = n';
end
n = n/norm(n);
Rot = n * n' + cos(theta) * (eye(3) - n * n') + sin(theta) * Skew_Sym_Mat(n);
end

function Mat = Skew_Sym_Mat(n)

n1 = n(1);
n2 = n(2);
n3 = n(3);

Mat = [0  -n3 n2;...
       n3 0   -n1;...
      -n2 n1  0];
end

