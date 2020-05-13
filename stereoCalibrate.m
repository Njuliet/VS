
I1 = imread('Left.jpg');%读取左右图片
I2 = imread('Right.jpg');
figure
imshowpair(I1, I2, 'montage');
title('Original Images');

%加载stereoParameters对象。
load('新matlab标定结果.mat');%加载你保存的相机标定的mat

%[J1, J2] = rectifyStereoImages(I1, I2, stereoParams);
[J1_full,J2_full] = rectifyStereoImages(I1,I2,stereoParams,... 
  'OutputView','full');%使用“完整”输出视图校正图像。
imshow(stereoAnaglyph(J1_full,J2_full));%显示“完整”输出视图的结果。
imwrite(J1_full, 'fullLeft.jpg');
imwrite(J2_full, 'fullRight.jpg');

[J1_valid,J2_valid] = rectifyStereoImages(I1,I2,stereoParams,... 
  'OutputView','valid');%使用“有效”输出视图校正图像。这最适合计算视差。
imshow(stereoAnaglyph(J1_valid,J2_valid));%显示“有效”输出视图的结果。
imwrite(J1_valid, 'validLeft.jpg');
imwrite(J2_valid, 'validRight.jpg');
%imshowpair(J1, J2, 'montage');
%title('Undistorted Images');
%figure; imshow(cat(3, J1(:,:,1), J2(:,:,2:3)), 'InitialMagnification', 50);%图像显示50%
% WL1 = abs(imfilter(rgb2gray(J1), fspecial('Laplacian'), 'replicate', 'conv'));
% WL2 = abs(imfilter(rgb2gray(J2), fspecial('Laplacian'), 'replicate', 'conv'));
% WL1(WL1(:)>=WL2(:)) = 1;WL1(WL1(:)<WL2(:)) = 0;
% WL2(WL1(:)>=WL2(:)) = 0;WL2(WL1(:)<WL2(:)) = 1;
% J_F(:,:,1) = J1(:,:,1).*WL1+J2(:,:,1).*WL2;
% J_F(:,:,2) = J1(:,:,2).*WL1+J2(:,:,2).*WL2;
% J_F(:,:,3) = J1(:,:,3).*WL1+J2(:,:,3).*WL2;
% figure; imshow(J_F, 'InitialMagnification', 50);%图像显示50%

