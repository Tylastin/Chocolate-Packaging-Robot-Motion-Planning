

function [thetas, gst_final_actual, x,y,z, orientations]= motion_planning_algorithm(gst_initial, gstf, tau, beta, position_tolerance,rotation_tolerance, theta_start, link_lengths,joint_axes,q_axes, type_joint, gst0)
%This function runs the Inverse kinematics based motion planning algorithm 
%Returns a sequence of angles as well as a sequence of x,y,z coordinates
%and orientation matrices for each intermediate position and the final
%position

    Ar0= Rot_to_Quat(gst_initial(1:3,1:3));
    p0 = [0;gst_initial(1:3,4)];
    Ad0= 0.5*QuaternionProduct(p0,Ar0);
    
    %Calculate gamma for initial position:
    g0_p = 2*QuaternionProduct(Ad0,QuaternionConjugate(Ar0));
    gamma0 = [g0_p(2:4);Ar0];
    
    

    
    Arf = Rot_to_Quat(gstf(1:3,1:3));
    pf=gstf;
    Adf = 0.5*QuaternionProduct([0;gstf(1:3,4)],Arf);
    
    
    
    %Setup loop conditions
    position_error=1000;
    rotation_error=1000;

    i =0;
    gamma_t = gamma0;
    Ar_t=Ar0;
    Ad_t=Ad0;
    thetas=[];
    thetas=[thetas; theta_start];
    theta_t=theta_start;
    gst_t=gst_initial;
    x=[];
    y=[];
    z=[];
    orientations=[];
    %Motion planning algorithm loop
    while (position_error>position_tolerance || rotation_error>rotation_tolerance)
        %Find next gamma(t+h) using screw linear interpolation
        [p,q] = DualQuaternionProduct(QuaternionConjugate(Ar_t),QuaternionConjugate(Ad_t),Arf,Adf);
        [p_power,q_power] = DualQuaternionPower(p,q,tau);
        [Ar_t_plus_h,Ad_t_plus_h]= DualQuaternionProduct(Ar_t,Ad_t,p_power,q_power);
        
        position_quaternion = 2*QuaternionProduct(Ad_t_plus_h,QuaternionConjugate(Ar_t_plus_h));%check
        
        gamma_t_plus_h = [position_quaternion(2:4);Ar_t_plus_h];
        
        
        Js = SpatialmanipJac(gst0,type_joint,joint_axes,q_axes,theta_t);
        J1 = [-Ar_t(2) Ar_t(1) -Ar_t(4) Ar_t(3);
            -Ar_t(3) Ar_t(4) Ar_t(1) -Ar_t(2);
            -Ar_t(4) -Ar_t(3) Ar_t(2) Ar_t(1);];%check
        
        
        p_hat = [0 -gamma_t(3) gamma_t(2);
            gamma_t(3) 0 -gamma_t(1);
            -gamma_t(2) gamma_t(1) 0];
        J2 = [eye(3) 2*p_hat*J1;zeros(3) 2*J1];
        B = Js'*inv(Js*Js')*J2;
        theta_t_plus_h = theta_t + beta.*(B*(gamma_t_plus_h-gamma_t));
       
        gst_t_plus_h = direct_kin(gst0,type_joint,joint_axes,q_axes,theta_t_plus_h);
        
        thetas= [thetas theta_t_plus_h];
        
        
        %Finding errors
        position_error = sqrt((gst_t_plus_h(1,4)-gstf(1,4))^2+(gst_t_plus_h(2,4)-gstf(2,4))^2+(gst_t_plus_h(3,4)-gstf(3,4))^2);
        Ar_t_plus_h = Rot_to_Quat(gst_t_plus_h(1:3,1:3));
        a = sqrt((Arf(1)-Ar_t_plus_h(1))^2+(Arf(2)-Ar_t_plus_h(2))^2+(Arf(3)-Ar_t_plus_h(3))^2+(Arf(4)-Ar_t_plus_h(4))^2);
        b = sqrt((Arf(1)+Ar_t_plus_h(1))^2+(Arf(2)+Ar_t_plus_h(2))^2+(Arf(3)+Ar_t_plus_h(3))^2+(Arf(4)+Ar_t_plus_h(4))^2);
        rotation_error= min(a,b);
        
        
        x= [x; gst_t_plus_h(1,4)];
        y= [y; gst_t_plus_h(2,4)];
        z= [z; gst_t_plus_h(3,4)];
        orientations = [orientations; gst_t_plus_h(1:3,1:3)];
        
        
        %Setup variables for next iteration
        theta_t = theta_t_plus_h;
        Ar_t = Ar_t_plus_h;
        p_t = [0;gst_t_plus_h(1:3,4)];
        Ad_t = Ad_t_plus_h;
        
        gst_t = gst_t_plus_h;
        gamma_t = gamma_t_plus_h;
        
        
        i= i+1;
        
        
        
    end
    
    gst_final_actual=gst_t_plus_h;

    
end
