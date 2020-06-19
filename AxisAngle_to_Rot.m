function R = AxisAngle_to_Rot(axis, angle)%Defining Function
w1=axis(1);%Taking Inputs
w2=axis(2);
w3=axis(3);
theta=angle;

R=[w1*w1*(1-cos(theta))+cos(theta) w1*w2*(1-cos(theta))-w3*sin(theta) w1*w3*(1-cos(theta))+w2*sin(theta);%Applying Relation
   w1*w2*(1-cos(theta))+w3*sin(theta) w2*w2*(1-cos(theta))+cos(theta) w2*w3*(1-cos(theta))-w1*sin(theta);
   w1*w3*(1-cos(theta))-w2*sin(theta) w2*w3*(1-cos(theta))+w1*sin(theta) w3*w3*(1-cos(theta))+cos(theta)];


end
