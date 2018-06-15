% VelocityCompensation

function [cpv]= VelocityCompensation(wie,wen)

         cpv =[       0              -2*wie(3)-wen(3)       2*wie(2)+wen(2) ;
                2*wie(3)+wen(3)             0              -2*wie(1)-wen(1) ;
               -2*wie(2)-wen(2)       2*wie(1)+wen(1)             0           ];

       
  