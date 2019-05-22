im1 = imread('imgs\6\test1.png');
pic1 = imread('pictures\trashcan.png');
pic2 = imread('pictures\trike.png');
pic3 = imread('pictures\plant.png');
pic4 = imread('pictures\dog.png');
pic5 = imread('pictures\pumpkin.png');
lab_im1 = rgb2lab(im1);
lab_pic1 = rgb2lab(pic1);
lab_pic2 = rgb2lab(pic2);
lab_pic3 = rgb2lab(pic3);
lab_pic4 = rgb2lab(pic4);
lab_pic5 = rgb2lab(pic5);

L_edges = 0:100;
a_edges = -86:98;
b_edges = -107:94;

[im1_L_counts,im1_L_binLocations,~] = histcounts(lab_im1(:,:,1),L_edges,'Normalization', 'probability');
[im1_a_counts,im1_a_binLocations,~] = histcounts(lab_im1(:,:,2),a_edges,'Normalization', 'probability');
[im1_b_counts,im1_b_binLocations,~] = histcounts(lab_im1(:,:,3),b_edges,'Normalization', 'probability');

[pic1_L_counts,pic1_L_binLocations,~] = histcounts(lab_pic1(:,:,1),L_edges,'Normalization', 'probability');
[pic1_a_counts,pic1_a_binLocations,~] = histcounts(lab_pic1(:,:,2),a_edges,'Normalization', 'probability');
[pic1_b_counts,pic1_b_binLocations,~] = histcounts(lab_pic1(:,:,3),b_edges,'Normalization', 'probability');

[pic2_L_counts,pic2_L_binLocations,~] = histcounts(lab_pic2(:,:,1),L_edges,'Normalization', 'probability');
[pic2_a_counts,pic2_a_binLocations,~] = histcounts(lab_pic2(:,:,2),a_edges,'Normalization', 'probability');
[pic2_b_counts,pic2_b_binLocations,~] = histcounts(lab_pic2(:,:,3),b_edges,'Normalization', 'probability');

[pic3_L_counts,pic3_L_binLocations,~] = histcounts(lab_pic3(:,:,1),L_edges,'Normalization', 'probability');
[pic3_a_counts,pic3_a_binLocations,~] = histcounts(lab_pic3(:,:,2),a_edges,'Normalization', 'probability');
[pic3_b_counts,pic3_b_binLocations,~] = histcounts(lab_pic3(:,:,3),b_edges,'Normalization', 'probability');

[pic4_L_counts,pic4_L_binLocations,~] = histcounts(lab_pic4(:,:,1),L_edges,'Normalization', 'probability');
[pic4_a_counts,pic4_a_binLocations,~] = histcounts(lab_pic4(:,:,2),a_edges,'Normalization', 'probability');
[pic4_b_counts,pic4_b_binLocations,~] = histcounts(lab_pic4(:,:,3),b_edges,'Normalization', 'probability');

[pic5_L_counts,pic5_L_binLocations,~] = histcounts(lab_pic5(:,:,1),L_edges,'Normalization', 'probability');
[pic5_a_counts,pic5_a_binLocations,~] = histcounts(lab_pic5(:,:,2),a_edges,'Normalization', 'probability');
[pic5_b_counts,pic5_b_binLocations,~] = histcounts(lab_pic5(:,:,3),b_edges,'Normalization', 'probability');

im1_L_binLocations(1) = [];
im1_a_binLocations(1) = [];
im1_b_binLocations(1) = [];

pic1_L_binLocations(1) = [];
pic1_a_binLocations(1) = [];
pic1_b_binLocations(1) = [];

pic2_L_binLocations(1) = [];
pic2_a_binLocations(1) = [];
pic2_b_binLocations(1) = [];

pic3_L_binLocations(1) = [];
pic3_a_binLocations(1) = [];
pic3_b_binLocations(1) = [];

pic4_L_binLocations(1) = [];
pic4_a_binLocations(1) = [];
pic4_b_binLocations(1) = [];

pic5_L_binLocations(1) = [];
pic5_a_binLocations(1) = [];
pic5_b_binLocations(1) = [];

show = false;

if show
    figure;
    subplot(231);
    plot(im1_L_binLocations,im1_L_counts);
    subplot(232);
    plot(im1_a_binLocations,im1_a_counts);
    subplot(233);
    plot(im1_b_binLocations,im1_b_counts);
    subplot(234);
    plot(pic1_L_binLocations,pic1_L_counts);
    subplot(235);
    plot(pic1_a_binLocations,pic1_a_counts);
    subplot(236);
    plot(pic1_b_binLocations,pic1_b_counts);

    figure;
    subplot(231);
    plot(im1_L_binLocations,im1_L_counts);
    subplot(232);
    plot(im1_a_binLocations,im1_a_counts);
    subplot(233);
    plot(im1_b_binLocations,im1_b_counts);
    subplot(234);
    plot(pic2_L_binLocations,pic2_L_counts);
    subplot(235);
    plot(pic2_a_binLocations,pic2_a_counts);
    subplot(236);
    plot(pic2_b_binLocations,pic2_b_counts);

    figure;
    subplot(231);
    plot(im1_L_binLocations,im1_L_counts);
    subplot(232);
    plot(im1_a_binLocations,im1_a_counts);
    subplot(233);
    plot(im1_b_binLocations,im1_b_counts);
    subplot(234);
    plot(pic3_L_binLocations,pic3_L_counts);
    subplot(235);
    plot(pic3_a_binLocations,pic3_a_counts);
    subplot(236);
    plot(pic3_b_binLocations,pic3_b_counts);

    figure;
    subplot(231);
    plot(im1_L_binLocations,im1_L_counts);
    subplot(232);
    plot(im1_a_binLocations,im1_a_counts);
    subplot(233);
    plot(im1_b_binLocations,im1_b_counts);
    subplot(234);
    plot(pic4_L_binLocations,pic4_L_counts);
    subplot(235);
    plot(pic4_a_binLocations,pic4_a_counts);
    subplot(236);
    plot(pic4_b_binLocations,pic4_b_counts);

    figure;
    subplot(231);
    plot(im1_L_binLocations,im1_L_counts);
    subplot(232);
    plot(im1_a_binLocations,im1_a_counts);
    subplot(233);
    plot(im1_b_binLocations,im1_b_counts);
    subplot(234);
    plot(pic5_L_binLocations,pic5_L_counts);
    subplot(235);
    plot(pic5_a_binLocations,pic5_a_counts);
    subplot(236);
    plot(pic5_b_binLocations,pic5_b_counts);
end

dL1 = pdist2(im1_L_counts,pic1_L_counts);
da1 = pdist2(im1_a_counts,pic1_a_counts);
db1 = pdist2(im1_b_counts,pic1_b_counts);
d1 = [da1 db1];

dL2 = pdist2(im1_L_counts,pic2_L_counts);
da2 = pdist2(im1_a_counts,pic2_a_counts);
db2 = pdist2(im1_b_counts,pic2_b_counts);
d2 = [da2 db2];

dL3 = pdist2(im1_L_counts,pic3_L_counts);
da3 = pdist2(im1_a_counts,pic3_a_counts);
db3 = pdist2(im1_b_counts,pic3_b_counts);
d3 = [da3 db3];

dL4 = pdist2(im1_L_counts,pic4_L_counts);
da4 = pdist2(im1_a_counts,pic4_a_counts);
db4 = pdist2(im1_b_counts,pic4_b_counts);
d4 = [da4 db4];

dL5 = pdist2(im1_L_counts,pic5_L_counts);
da5 = pdist2(im1_a_counts,pic5_a_counts);
db5 = pdist2(im1_b_counts,pic5_b_counts);
d5 = [da5 db5];



d = [mean(d1);mean(d2);mean(d3);mean(d4);mean(d5)]
[m,i] = min(d)