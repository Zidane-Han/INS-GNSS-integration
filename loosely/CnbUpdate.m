function Cnb=CnbUpdate(Qt)

        Cnb(1,1)= Qt(1)^2+Qt(2)^2-Qt(3)^2-Qt(4)^2;    
        Cnb(2,2)= Qt(1)^2-Qt(2)^2+Qt(3)^2-Qt(4)^2;
        Cnb(3,3)= Qt(1)^2-Qt(2)^2-Qt(3)^2+Qt(4)^2;          
        Cnb(1,2)= 2*(Qt(2)*Qt(3)-Qt(1)*Qt(4));         
        Cnb(1,3)= 2*(Qt(2)*Qt(4)+Qt(1)*Qt(3)); 
        Cnb(2,1)= 2*(Qt(2)*Qt(3)+Qt(1)*Qt(4));
        Cnb(2,3)= 2*(Qt(3)*Qt(4)-Qt(1)*Qt(2));        
        Cnb(3,1)= 2*(Qt(2)*Qt(4)-Qt(1)*Qt(3));        
        Cnb(3,2)= 2*(Qt(3)*Qt(4)+Qt(1)*Qt(2));             