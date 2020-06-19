function [q3a,q3b] = DualQuaternionProduct(q1a,q1b,q2a,q2b)
    q3a = QuaternionProduct(q1a,q2a);
    q3b = QuaternionProduct(q1b,q2a)+QuaternionProduct(q1a,q2b);
end
