function [plan2, width] = extend_y(plan)
% Extension of the width of the map.
% plan : intial map
% plan2 : extended map
% width : new length

[a,b] = size(plan);
plan2 = zeros(a, b*2, 'uint8');
low = round(b/2);
% Complete plan2 with the element in plan
plan2(:,low+1:b+low) = plan(:,:);
width = a*2;
