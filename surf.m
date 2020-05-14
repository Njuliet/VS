%��ȡͼƬ
sceneImageL = imread('fullLeft.jpg');
% sceneImageL = imread('Left.jpg');%δУ����ͼ��ǵ�ǳ���
LeftImage=rgb2gray(sceneImageL);%ͼƬתΪ�Ҷ�ͼ
% figure;%����ͼ��
% imshow(sceneImage);
% title('Image of a Left');
sceneImageR= imread('fullRight.jpg');
% sceneImageR= imread('Right.jpg');
RightImage=rgb2gray(sceneImageR);%ͼƬתΪ�Ҷ�ͼ
% figure;
% imshow(sceneImage1);
% title('Image of a Right');
%���������
scenePoints = detectSURFFeatures(LeftImage);
scenePoints1 = detectSURFFeatures(RightImage);


%���ӻ���Ŀ��ͼ�����ҵ�����ǿ������
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
%��ȡ������ʾ��
[LeftFeatures,LeftPoints] = extractFeatures(LeftImage,scenePoints); 
[RightFeatures,RightPoints] = extractFeatures(RightImage,scenePoints1);
%ʹ����������ƥ��Ҫ�ء�
Pairs = matchFeatures(LeftFeatures,RightFeatures);
%��ʾ�ƶ�ƥ��Ĺ��ܡ�
matchedLeftPoints = LeftPoints(Pairs(:, 1), :);
matchedRightPoints = RightPoints(Pairs(:, 2), :);

leftCoord=matchedLeftPoints.Location;%�洢����ƥ�������������
rightCoord=matchedRightPoints.Location;
%leftCoord ��rightCoord���Ǿ���ɸѡ��ƥ���
figure;
showMatchedFeatures(LeftImage, RightImage, matchedLeftPoints, ...
    matchedRightPoints, 'montage');
title('����ƥ��ĵ㣨�����쳣ֵ��');
%estimateGeometricTransform������ƥ�����صı任��ͬʱ�����쳣ֵ������ת��ʹ���ǿ��Զ�λ�����еĶ���
[tform,inlierLeftPoints,inlierRightPoints] = ... 
    estimateGeometricTransform(matchedLeftPoints,matchedRightPoints,'affine');
%��ʾ��ɾ���쳣ֵ��ƥ����
figure;
showMatchedFeatures(LeftImage, RightImage, inlierLeftPoints, ...
    inlierRightPoints, 'montage');
title('Matched Points (Inliers Only)');
leftCoord1=inlierLeftPoints.Location;%�洢һ��ƥ�������������
rightCoord1=inlierRightPoints.Location;