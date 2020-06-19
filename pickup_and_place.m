%Baxter Arm pick and place
%Description takes an initial position and any number of dropoff locations in configuration form 
% Arm starts at in initial position and moves to the first picup location,
% then moves back to the initial position. The arm repeats this process
% until all pickup locations have been reached and the arm returns to the
% inital position. This script provides a complete set of joint angles stored in the thetas variable
% to program the baxter arm and
% a graph to illustrate the trajectory over time. 

%Problem setup:

l=[270.35 69 364.35 69 374.29 10 229.525]/1000;%m
%Joint axes 
w1=[0; 0; 1];
w2=[-1; 0; 0];
w3=[0; 1; 0];
w4=w2;
w5=w3;
w6=w2;
w7=w3;
joint_axes=[w1 w2 w3 w4 w5 w6 w7];

%points on axes with the world frame origin at the first joint 
q1=[0; 0; 0];
q2=[0; l(2); l(1)];
q3=q2;
q4=[0; l(2)+l(3); l(1)-l(4)];
q5=q4;
q6=[0; l(2)+l(3)+l(5); l(1)-l(4)-l(6)];
q7=q6;
q_axes=[q1 q2 q3 q4 q5 q6 q7];
type_joint=['R','R','R','R','R','R','R'];

%Setup initial position corresponding to the arm fully extended in the
%direction of the positive y axis 
pst0=  transpose([0 l(2)+l(3)+l(5)+l(7) l(1)-l(4)-l(6)]);

rst0=  [1 0 0;
        0 1 0;
        0 0 1];

gst0 = [rst0 pst0;
    0 0 0 1];
theta_pickup_configuration = [0; -80; 0; 100; 0; 20; 0];
theta_pickup_configuration = deg2rad(theta_pickup_configuration);


[gst_pickup, transform_upto_joint] = direct_kin(gst0, type_joint, joint_axes, q_axes, theta_pickup_configuration);


%Dropoff configurations are set to 1cm above the center of the chocolate slot. Chocolates are spheres so
%orientation accuracy is not required. Chocolate slots are arranged in a
%grid configuration as shown by the animation. grid units are in meters;





gst_dropoffs = [];% array contains all dropoff configurations
chocolate_slot_1_dropoff_configuration=[
    1 0 0 0 ;
    0 1 0 0.8; 
    0 0 1 0.0;
    0 0 0 1];
chocolate_slot_2_dropoff_configuration=[
    1 0 0 0 ;
    0 1 0 0.84;
    0 0 1 0.0;
    0 0 0 1];
chocolate_slot_3_dropoff_configuration=[
    1 0 0 0 ;
    0 1 0 0.88;
    0 0 1 0.0;
    0 0 0 1];

chocolate_slot_4_dropoff_configuration=[
    1 0 0 -0.02 ;
    0 1 0 0.8; 
    0 0 1 0.0;
    0 0 0 1];
chocolate_slot_5_dropoff_configuration=[
    1 0 0 -0.02 ;
    0 1 0 0.84;
    0 0 1 0.0;
    0 0 0 1];
chocolate_slot_6_dropoff_configuration=[
    1 0 0 -0.02 ;
    0 1 0 0.88;
    0 0 1 0.0;
    0 0 0 1];


gst_dropoffs= [chocolate_slot_1_dropoff_configuration chocolate_slot_2_dropoff_configuration chocolate_slot_3_dropoff_configuration chocolate_slot_4_dropoff_configuration chocolate_slot_5_dropoff_configuration chocolate_slot_6_dropoff_configuration];


gst_actual_pickup=gst_pickup; %Assuming the robot starts at ideal pickup location 

%Step size paramters for motion planning algorithm(should be selected
%through trial and error): 
tau = 0.01;
beta=1; 
position_tolerance= 0.005;
rotation_tolerance=1000;
x_array=[];
y_array=[];
z_array=[];
thetas=[];% Complete sequence of joint angle configurations to complete the task 
thetas=[thetas theta_pickup_configuration];



%Animation graph setup 

gamma0= gst_pickup(1:3,4);
plot3(gamma0(1),gamma0(2),gamma0(3),'*');
hold on
text(gamma0(1),gamma0(2),gamma0(3),'  Ideal Pickup Position');
hold on
for i=1:size(gst_dropoffs,2)/4
    gst_desired_dropoff=gst_dropoffs(:,(i-1)*4+1:i*4);
    pf =gst_desired_dropoff;
    plot3(pf(1,4),pf(2,4),pf(3,4),'*');
    hold on
    text(pf(1,4),pf(2,4),pf(3,4),' Desired Dropoff');
    hold on
    
end

pause(2);


%Animation loop shows trajectory from pickup location to choco
for(i=1:size(gst_dropoffs,2)/4)
    gst_desired_dropoff=gst_dropoffs(:,(i-1)*4+1:i*4);
    [theta, gst_actual_dropoff, x,y,z,orientations]= motion_planning_algorithm(gst_actual_pickup, gst_desired_dropoff, tau, beta,position_tolerance, rotation_tolerance,thetas(:,end),l,joint_axes,q_axes,type_joint,gst0);
    thetas=[thetas theta];
    x_array=[x_array; x];
    y_array=[y_array; y];
    z_array=[z_array; z];
    grid on
    comet3(x,y,z);
    hold on
    [theta, gst_actual_pickup, x,y,z,orientations]= motion_planning_algorithm(gst_actual_dropoff, gst_pickup, tau, beta, position_tolerance, rotation_tolerance,thetas(:,end),l,joint_axes,q_axes,type_joint,gst0);
    thetas=[thetas theta];
    x_array=[x_array; x];
    y_array=[y_array; y];
    z_array=[z_array; z];      
    comet3(x,y,z);
    hold on 

end 
   

%joint angles to program baxter to perform the movement shown in the animation. are stored in the the thetas variable;
thetas