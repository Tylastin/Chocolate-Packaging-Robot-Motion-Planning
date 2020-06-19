function [product ] = DualVectorProduct(V1,V2)

    [a b] = DualNumberProduct(V1,V2(1,:));
    [c d] = DualNumberProduct(V1,V2(2,:));
    [e f] = DualNumberProduct(V1,V2(3,:));
    product = [[a b];[c d];[e f]];

end