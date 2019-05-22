tic;
trainFolder = fullfile('imgs');
train = imageSet(trainFolder,'recursive');
bag = bagOfFeatures(train,'verbose',false);
categoryClassifier = trainImageCategoryClassifier(train,bag,'verbose',false);
list = dir('pictures/*.png');
cd pictures;
score_mat = zeros(length(list));
for i=1 : length(list)
    img = imread(fullfile(list(i).name));
    [labelIdx, score] = predict(categoryClassifier,img);
    score_mat(i,:) = score;
end
cd ..;

per = perms(1:5);
to_add = 0:5:20;
best_score = - Inf;
score_mat = score_mat.';
for i = 1 : size(per,1)
    score = sum(score_mat(per(i,:) + to_add));
    if( score > best_score )
        best_score = score;
        result = per(i,:);
    end
end
toc