function Q = Rot_to_Quat(R)
% We should first check that R is a rotation matrix

r11 = R(1,1);
r22 = R(2,2);
r33 = R(3,3);
rdiag = [r11; r22; r33; 1];

coeff_mat = 1/4*[1 1 1 1; 1 -1 -1 1; -1 1 -1 1; -1 -1 1 1];
 
quat_square = coeff_mat*rdiag;

[qi, i] = max(quat_square);
qi = sqrt(qi);
%qi = -sqrt(qi); % Alternate solution
inv_qi = 1/(4*qi); 

if i==1
    q0 = qi;
    q1 = inv_qi*(R(3,2) - R(2,3));
    q2 = inv_qi*(R(1,3) - R(3,1));
    q3 = inv_qi*(R(2,1) - R(1,2));
end
if i == 2
    q0 = inv_qi*(R(3,2) - R(2,3));
    q1 = qi;
    q2 = inv_qi*(R(1,2) + R(2,1));
    q3 = inv_qi*(R(1,3) + R(3,1));
end
if i == 3
    q0 = inv_qi*(R(1,3) - R(3,1));
    q1 = inv_qi*(R(1,2) + R(2,1));
    q2 = qi;
    q3 = inv_qi*(R(3,2) + R(2,3));
end
if i == 4
    q0 = inv_qi*(R(2,1) - R(1,2));
    q1 = inv_qi*(R(1,3) + R(3,1));
    q2 = inv_qi*(R(3,2) + R(2,3));
    q3 = qi;
end
Q = [q0; q1; q2; q3];
end