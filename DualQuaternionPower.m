function [q1a,q1b] = DualQuaternionPower(qa,qb,power)

    theta = 2*acos(qa(1));
    l = [0;qa(2:4)/(sin(theta/2));];
    
    P = 2*QuaternionProduct(qb,QuaternionConjugate(qa));
    d = dot(P(2:4),l(2:4));
    m = [0;0.5*(cross(P(2:4),l(2:4))+(P(2:4)-d*l(2:4))*cot(theta/2))];
    
    V1 = [sin(power*theta/2) (power*d/2)*cos(power*theta/2)];
    V2 = [l(2:4) m(2:4)];
    imaginary = DualVectorProduct(V1,V2);
    q1a = [cos(power*theta/2);imaginary(:,1)];
    q1b = [-(power*d/2)*sin(power*theta/2);imaginary(:,2)];
end
