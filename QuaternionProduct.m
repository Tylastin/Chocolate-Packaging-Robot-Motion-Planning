function Quat = QuaternionProduct(p,r)

    
    q0 = p(1)*r(1)-p(2)*r(2)-p(3)*r(3)-p(4)*r(4);
    q1 = p(1)*r(2)+p(2)*r(1)+p(3)*r(4)-p(4)*r(3);
    q2 = p(1)*r(3)-p(2)*r(4)+p(3)*r(1)+p(4)*r(2);
    q3 = p(1)*r(4)+p(2)*r(3)-p(3)*r(2)+p(4)*r(1);
    Quat = [q0;q1;q2;q3];

end