function adjoint= AdjointOfg(g)
    g_cell = num2cell(g);
    [r11, r21, r31, z1, r12, r22, r32, z2, r13, r23, r33,z3, p1, p2, p3,o1] = g_cell{:};

    R= [r11 r12 r13 ;
        r21 r22 r23 ;
        r31 r32 r33 ];
    
    
    p_hat = [0 -p3 p2; 
        p3 0 -p1;
        -p2 p1 0];

    adjoint = [R p_hat*R;
            zeros(3) R];
        
end