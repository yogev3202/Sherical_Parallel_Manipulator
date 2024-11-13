function dEA = W2dEA(EA,W)


    x1 = EA(1);
    x2 = EA(2);
    x3 = EA(3);

    Em = [ 0  , -sin(x1) , cos(x1)*cos(x2);
           0  ,  cos(x1) , sin(x1)*cos(x2);
           1  ,    0      ,    -sin(x2)]; 

    dEA = Em^(-1)*W;

end