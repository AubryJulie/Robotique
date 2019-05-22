cd imgs;
imgs = zeros(301,410,3,5);
grays = zeros(301,410,5);
for i = 2 : 6
    fold = int2str(i);
    cd(fold);
    gray(:,:,i-1) = rgb2gray(imread('test1.png')); 
    cd ..;
end
cd ..;
img = round(mean(gray,3));
figure;
imshow(gray(:,:,1));
figure;
imshow(gray(:,:,2));
figure;
imshow(gray(:,:,3));
figure;
imshow(gray(:,:,4));
figure;
imshow(gray(:,:,5));
figure;
imshow(img);