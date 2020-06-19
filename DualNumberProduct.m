function [ real, imaginary ] = DualNumberProduct(D1,D2)
    real = D1(1)*D2(1);
    imaginary = D1(1)*D2(2)+D2(1)*D1(2);
end
