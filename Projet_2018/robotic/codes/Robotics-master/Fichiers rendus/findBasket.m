function [basket] = findBasket(object,inst)
    for i = 1 : length(inst)
       if(strcmp(object.shape,inst(i).shape)&& strcmp(object.color,inst(i).colorname))
          basket = inst(i).picture;
       end
    end
end