%%Function that finds the different rooms, as well as the baskets and the
%%deposit, it also returns a path were the robot can navigate.

function [baskets,tables,r,p_map_c,p_map]=locate(value_map, init_pos)
  
  %find circle an assiciate each one to a either a basket or a table
  %containing element.
  %TODO compute efficiently the radius of the disk w.r.t to the resolution
  circles = imopen(imclose(value_map,strel('disk',3)),strel('disk',3));
  %TODO better use of imfindcircles
  [c_, r, ~] = imfindcircles(circles,[1 50]);
  r_ = r;
  r_(:) = mean(r);
  dist = sqrt((c_(:,1)-init_pos(1)).^2+(c_(:,2)-init_pos(2)).^2);
  [~,i] = min(dist);
  tables(2) = struct('center',[],'path',[],'ThreeDmap',[]);
  baskets(length(r)-2) = struct('center',[],'path',[],'object',[],'picture',[]);
  table = zeros(2,2);
  table(1,:)= c_(i,:);
  c_(i,:)=[];
  dist(i) = [];
  [~,i] = min(dist);
  table(2,:)=c_(i,:);
  c_(i,:)= [];
  basket = c_;
  %Make the difference between each room.
  %TODO don't hardcode those value
  hori = imopen(value_map,strel('rectangle',[1,17]));
  vert = imopen(value_map,strel('rectangle',[17,1]));
  tmp = hori + vert;
  tmp(tmp == 2) = 1;
  %Todo don't hardcode those value
  hori = imclose(tmp,strel('rectangle',[1,20]));
  vert = imclose(tmp,strel('rectangle',[20,1]));
  room = hori + vert;
  room(room==2) = 1;
  room = watershed(room);  
  v_map = room;
  v_map(v_map~=0)=2;
  v_map(v_map==0)=1;
  v_map(v_map==2)=0;
  v_map(v_map==1 & value_map==0)=0;
  v_map(size(v_map,1),:)=1;
  v_map(1,:)=1;
  v_map(:,size(v_map,1))=1;
  v_map(:,1)=1;
  p_map = idilate(v_map,kcircle(3));
  
  ang = linspace(0,2*pi,28);
  % TODO change the 4 with a value relative to the radius
  r = r_(1) + 12;
  b_path = cell(size(basket,1),1);
  for i = 1 : size(basket,1)
      x_ = r*cos(ang) + round(basket(i,1));
      y_ = r*sin(ang) + round(basket(i,2));
      in_room = room(round(basket(i,2)),round(basket(i,1)));
      x=round(x_);
      y=round(y_);
      p_x = [];
      p_y = [];
      for j=1:length(x_)
         if(y(j)>0 && y(j)<=size(p_map,1) && x(j)>0 && x(j)<=size(p_map,2))
            if(room(y(j),x(j))==in_room && p_map(y(j),x(j))==0)
                p_x = [p_x; x_(j)];
                p_y = [p_y; y_(j)];
            end
         end
      end
      % We try to estimate the position of the object we want to photograph
      x_m = mean(p_x);
      y_m = mean(p_y);
      opp = (y_m-basket(i,2));
      adj = (x_m-basket(i,1));
      hyp = sqrt(opp^2+adj^2);
      ang2 = atan(opp./adj);      
      if((adj/hyp)<0)
         ang2 = ang2 + pi;
      end
      ang2 = ang2 +pi;
      x_obj = r_(1)*cos(ang2) + basket(i,1);
      y_obj = r_(1)*sin(ang2) + basket(i,2);
      %fill the structure.
      baskets(i).center = basket(i,:);
      baskets(i).path = [x_m y_m];
      baskets(i).object = [x_obj y_obj];
  end
  
  p_map = v_map;
  
  for i = 1 : size(table,1)
      x = (r_(1)+5)*cos(pi/2) + table(i,1);
      x_ = r_(1)*cos(ang) + table(i,1);
      y = (r_(1)+5)*sin(pi/2) + table(i,2);
      y_ = r_(1)*sin(ang) + table(i,2);
      tables(i).center = table(i,:);
      tables(i).path = [x y];
      %draw the table on the map
      v_map(round(y_),round(x_)) = 1;
  end
  r = r_(1);
  p_map_c = v_map;
    
end