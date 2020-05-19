clc;
clear;
left=double(rgb2gray(imread('imL.png')));
right=double(rgb2gray(imread('imR.png')));

[m ,n]=size(left); %行数（垂直分辨率）、列数（水平分辨率）

r=3  ;       %初始窗口边长的一半
imgn=zeros(m,n); %左视差图
imgn1=zeros(m,n); %右视差图

flag1=zeros(m,n);  %用来标志左图中的像素点是否访问过  图像坐标系是以图像的左上角为原点
flag2=zeros(m,n);  %右图   部分边界会有问题
% flag1=zeros(400,500);  %用来标志左图中的像素点是否访问过  图像坐标系是以图像的左上角为原点
% flag2=zeros(400,500);

yuzhi=3000;
%对每一对特征点（x1,y1）(x2,y2) 来说
%点的横坐标范围是0-n,纵坐标是0-m
step=5;%窗口增大步长
[leftCoord1,rightCoord1]=mysurf(imread('imL.png'),imread('imR.png'),r);%得到左右图中特征点的横纵坐标
leftsort=sortrows(leftCoord1);
rightsort=sortrows(rightCoord1);
leftCoord1x=leftsort(:,1);
leftCoord1y=leftsort(:,2);
rightCoord1x=rightsort(:,1);
rightCoord1y=rightsort(:,2);
%   figure;%创建一个新的窗口
%   hold on;
% x1,y1是左中心坐标，x2,y2是右中心坐标，step是每次往外扩的步长，diff是视差的最大值,flag1是左图的标志矩阵，flag2是右图的标志矩阵
while r <= max(m,n)
for index=1:length(leftCoord1x)
    x1=leftCoord1x(index);
    y1=leftCoord1y(index);
    x2=rightCoord1x(index);
    y2=rightCoord1y(index);
    %     plotSquare( x1,y1,r );
    %     plotSquare( x2,y2,r );
    [flag1,flag2,imgn] = window( left,right,x1,y1,x2,y2,r,flag1,flag2,n,m,imgn);
end
     r=r+step;
 end
% title('左图匹配区域');
% axis equal;
% figure;%创建一个新的窗口
%  hold on;
% for index=1:length(leftCoord1x)
%     x1=leftCoord1x(index);
%     y1=leftCoord1y(index);
%     x2=rightCoord1x(index);
%     y2=rightCoord1y(index);
%     plotSquare( x2,y2,r );
%     [flag1,flag2,imgn1] = window1( left,right,x1,y1,x2,y2,r,yuzhi,flag1,flag2,n,m,imgn1);
% end
% title('右图匹配区域');
% axis equal;
figure;%创建一个新的窗口
 axis ij ;
imshow(imgn,[]);
title('左视差图');
% figure;%创建一个新的窗口
% imshow(imgn1,[]);
% title('右视差图');


