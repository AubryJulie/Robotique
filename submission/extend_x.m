function [plan2, len] = extend_x(plan)
% Extension of the length of the map.
% plan : intial map
% plan2 : extended map
% len : new length

[a,b] = size(plan);
plan2 = zeros(a*2, b, 'uint8');
low = round(a/2);
% Complete plan2 with the element in plan
plan2(low+1:a+low,:) = plan(:,:);
len = a*2;
