function [bwIm] = colorTresh(image,tresh)
    r = image(:,:,1);
    b = image(:,:,2);
    g = image(:,:,3);
    if(tresh(1)>128)
        r(r<=tresh(1)) = 0;
        r(r>tresh(1)) = 1;
    else
        r(r>tresh(1)) = 0;
        r(r<=tresh(1)) = 1;
    end
    
    if(tresh(2)>128)
         b(b <= tresh(2)) = 0;
         b(b > tresh(2)) = 1;
    else
         b(b > tresh(2)) = 0;
         b(b <= tresh(2)) = 1;
    end
    
    if(tresh(3)>128)
        g(g <= tresh(3)) = 0;
        g(g > tresh(3)) = 1;
    else
        g(g > tresh(3)) = 0;
        g(g <= tresh(3)) = 1;
    end
    
    
   
    
    
    r = logical(r);
    b = logical(b);
    g = logical(g);
    bwIm = r & g & b;
end