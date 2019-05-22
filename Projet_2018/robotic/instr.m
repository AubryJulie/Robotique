formBlock = 'cylinder';
colorName = 'blue';

basketGoal = [

    70    10
    42    34
    54    58
    70    58
    18    18];

basketId = [

     1     5     4     2     3];

load('instructions.mat');
load('images.mat');
load('categoryClassifier1.mat');


for i=1:length(inst)
if strcmp (formBlock, inst(i).shape)
    if strcmp (colorName,inst(i).colorname)
        image = imread(fullfile(inst(i).picture));
        [label, scores] = predict(categoryClassifier1, image);
        for j = 1:length(basketId)
            if basketId(j) == label
                break;
            end
        end
        NewGoal = basketGoal(i,:);
    end
end
end

