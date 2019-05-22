function [colorname] = objectColor(imageROI)
    colorTresh(1).mask = [220 90 90];
    colorTresh(1).name = 'red';
    
    colorTresh(2).mask = [220 220 80];
    colorTresh(2).name = 'yellow';
    
    colorTresh(3).mask = [90 220 90];
    colorTresh(3).name = 'green';
    
    colorTresh(4).mask = [80 80 220];
    colorTresh(4).name = 'blue';
    
    colorTresh(5).mask = [220 80 220];
    colorTresh(5).name = 'purple';
    
    colorTresh(6).mask = [240 240 240];
    colorTresh(6).name = 'white';

    for i = 1 : 6
        r = imageROI(:,:,1);
        b = imageROI(:,:,2);
        g = imageROI(:,:,3);
        tresh = colorTresh(i).mask;
        if(tresh(1)>128)
            r(r<=tresh(1)) = 0;
            r(r>tresh(1)) = 1;
        else
            r(r<=tresh(1)) = 1;
            r(r>tresh(1)) = 0;
        end
        if(tresh(2)>128)
             b(b <= tresh(2)) = 0;
             b(b > tresh(2)) = 1;
        else
             b(b <= tresh(2)) = 1;
             b(b > tresh(2)) = 0;
        end
        if(tresh(3)>128)
            g(g <= tresh(3)) = 0;
            g(g > tresh(3)) = 1;
        else
            g(g <= tresh(3)) = 1;
            g(g > tresh(3)) = 0;
        end
        r = logical(r);
        b = logical(b);
        g = logical(g);
        bwIm = r & g & b;
        if(abs(mean(bwIm) - 1) < 0.001)
            colorname = colorTresh(i).name;
            break;
        end
    end
end