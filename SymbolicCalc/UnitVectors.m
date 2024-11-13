function [vi0,wi,Vim,Wim,Mim,Nim] = UnitVectors(e,q)

   R2D = 180/pi;
   D2R = pi/180;

   a1 = 65*D2R;a2 = 60*D2R;b1 = 0*D2R;b2 = 110*D2R;

   Sa1 = sin(a1); Sa2 = sin(a2);
   Sb1 = sin(b1); Sb2 = sin(b2);
   Ca1 = cos(a1); Ca2 = cos(a2);
   Cb1 = cos(b1); Cb2 = cos(b2);

   vi0 = [Sb2*sin(e);Sb2*cos(e);Cb2];

   wi = [sin(e)*(Sb1*Ca1+Cb1*Sa1*cos(q)) - cos(e)*Sa1*sin(q);
         cos(e)*(Sb1*Ca1+Cb1*Sa1*cos(q)) + sin(e)*Sa1*sin(q);
         -Cb1*Ca1 + Sb1*Sa1*cos(q)]; 

   Vim = [-crossVM(vi0) , vi0; -vi0' , 0];
   Wim = [crossVM(wi) , wi; -wi' , 0];
   Mim = (1/Ca2)*(Wim'*Vim);
   Nim = 0.5*Mim + (1 - 0.5*Ca2)*eye(4);

end

