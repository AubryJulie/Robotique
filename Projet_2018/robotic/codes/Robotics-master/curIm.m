cd imgs;
for j = 1 : 5
    fold = int2str(j);
    cd(fold)
    list = dir('test*.png');
    for i=1 : length(list)
        img = imread(fullfile(list(i).name));
        img = img(1:round(end/1.7),1:round(4*end/5),:);
        imwrite(img,list(i).name);
    end
    cd ..;
end
cd ..;