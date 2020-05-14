%读取图片
sceneImageL = imread('fullLeft.jpg');
%  sceneImageL = imread('im2.png');
% sceneImageL = imread('Left.jpg');%未校正的图像角点非常少
LeftImage=rgb2gray(sceneImageL);%图片转为灰度图
% figure;%建立图形
% imshow(sceneImage);
% title('Image of a Left');
sceneImageR= imread('fullRight.jpg');
% sceneImageR= imread('im6.png');
% sceneImageR= imread('Right.jpg');
RightImage=rgb2gray(sceneImageR);%图片转为灰度图
% figure;
% imshow(sceneImage1);
% title('Image of a Right');
%检测特征点
scenePoints = detectSURFFeatures(LeftImage);
scenePoints1 = detectSURFFeatures(RightImage);


%可视化在目标图像中找到的最强特征点
figure;
imshow(LeftImage);
title('300 Strongest Feature Points from Left');
hold on;
plot(selectStrongest(scenePoints, 300));
figure;
imshow(RightImage);
title('300 Strongest Feature Points from Right');
hold on;
plot(selectStrongest(scenePoints1, 300));
%提取功能提示符
[LeftFeatures,LeftPoints] = extractFeatures(LeftImage,scenePoints); 
[RightFeatures,RightPoints] = extractFeatures(RightImage,scenePoints1);
%使用其描述符匹配要素。
Pairs = matchFeatures(LeftFeatures,RightFeatures);
%显示推定匹配的功能。
matchedLeftPoints = LeftPoints(Pairs(:, 1), :);
matchedRightPoints = RightPoints(Pairs(:, 2), :);

leftCoord=matchedLeftPoints.Location;%存储可能匹配的特征点坐标
rightCoord=matchedRightPoints.Location;
%leftCoord 和rightCoord就是经过筛选的匹配点
figure;
showMatchedFeatures(LeftImage, RightImage, matchedLeftPoints, ...
    matchedRightPoints, 'montage');
title('可能匹配的点（包括异常值）');
%estimateGeometricTransform计算与匹配点相关的变换，同时消除异常值。这种转换使我们可以定位场景中的对象。
[tform,inlierLeftPoints,inlierRightPoints] = ... 
    estimateGeometricTransform(matchedLeftPoints,matchedRightPoints,'affine');
%显示已删除异常值的匹配点对
figure;
showMatchedFeatures(LeftImage, RightImage, inlierLeftPoints, ...
    inlierRightPoints, 'montage');
title('Matched Points (Inliers Only)');

% leftCoord1=inlierLeftPoints.Location;%存储一定匹配的特征点坐标
leftCoord1=matchedLeftPoints.Location;%存储可能匹配的特征点坐标
rightCoord1=inlierRightPoints.Location;

leftdistance=pdist(leftCoord1);%共从1累加到size(leftCoord1)的总数列
%size(leftCoord1,1)可得矩阵的行数，也就是一定匹配的特征点的个数

maxdistance=max(leftdistance);
mindistance=min(leftdistance);

figure;%创建一个新的窗口
imshow(LeftImage);
hold on;%保留当前坐标轴中的绘图，从而使新添加到坐标轴的绘图不会删除现有绘图
for i=1:size(leftCoord1,1)%可得矩阵的行数，也就是一定匹配的特征点的个数
    a1=leftCoord1(i,:);%第一个圆心坐标
    a2=a1(:,1);  %坐标的x值
    a3=a1(:,2);  %坐标的y值
    plot1(a2,a3,maxdistance/8);
    hold on;
end

