function  y = crossVM(x)
if (size(x)== [3,1] )
   
  y = [ 0   -x(3)  x(2)
     
       x(3)   0    -x(1)

      -x(2)  x(1)   0    ];

elseif (size(x)==[3,3]) 
   y = [x(3,2);x(1,3);x(2,1)];
else
   disp('error in the input dimensions ');
end


