
I1 = imread('Left.jpg');%��ȡ����ͼƬ
I2 = imread('Right.jpg');
figure
imshowpair(I1, I2, 'montage');
title('Original Images');

%����stereoParameters����
load('��matlab�궨���.mat');%�����㱣�������궨��mat

%[J1, J2] = rectifyStereoImages(I1, I2, stereoParams);
[J1_full,J2_full] = rectifyStereoImages(I1,I2,stereoParams,... 
  'OutputView','full');%ʹ�á������������ͼУ��ͼ��
imshow(stereoAnaglyph(J1_full,J2_full));%��ʾ�������������ͼ�Ľ����
imwrite(J1_full, 'fullLeft.jpg');
imwrite(J2_full, 'fullRight.jpg');

[J1_valid,J2_valid] = rectifyStereoImages(I1,I2,stereoParams,... 
  'OutputView','valid');%ʹ�á���Ч�������ͼУ��ͼ�������ʺϼ����Ӳ
imshow(stereoAnaglyph(J1_valid,J2_valid));%��ʾ����Ч�������ͼ�Ľ����
imwrite(J1_valid, 'validLeft.jpg');
imwrite(J2_valid, 'validRight.jpg');
%imshowpair(J1, J2, 'montage');
%title('Undistorted Images');
%figure; imshow(cat(3, J1(:,:,1), J2(:,:,2:3)), 'InitialMagnification', 50);%ͼ����ʾ50%
% WL1 = abs(imfilter(rgb2gray(J1), fspecial('Laplacian'), 'replicate', 'conv'));
% WL2 = abs(imfilter(rgb2gray(J2), fspecial('Laplacian'), 'replicate', 'conv'));
% WL1(WL1(:)>=WL2(:)) = 1;WL1(WL1(:)<WL2(:)) = 0;
% WL2(WL1(:)>=WL2(:)) = 0;WL2(WL1(:)<WL2(:)) = 1;
% J_F(:,:,1) = J1(:,:,1).*WL1+J2(:,:,1).*WL2;
% J_F(:,:,2) = J1(:,:,2).*WL1+J2(:,:,2).*WL2;
% J_F(:,:,3) = J1(:,:,3).*WL1+J2(:,:,3).*WL2;
% figure; imshow(J_F, 'InitialMagnification', 50);%ͼ����ʾ50%

