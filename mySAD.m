clc;
clear;
left=double(rgb2gray(imread('imL.png')));
right=double(rgb2gray(imread('imR.png')));

[m ,n]=size(left); %��������ֱ�ֱ��ʣ���������ˮƽ�ֱ��ʣ�

r=3  ;       %��ʼ���ڱ߳���һ��
imgn=zeros(m,n); %���Ӳ�ͼ
imgn1=zeros(m,n); %���Ӳ�ͼ

flag1=zeros(m,n);  %������־��ͼ�е����ص��Ƿ���ʹ�  ͼ������ϵ����ͼ������Ͻ�Ϊԭ��
flag2=zeros(m,n);  %��ͼ   ���ֱ߽��������
% flag1=zeros(400,500);  %������־��ͼ�е����ص��Ƿ���ʹ�  ͼ������ϵ����ͼ������Ͻ�Ϊԭ��
% flag2=zeros(400,500);

yuzhi=3000;
%��ÿһ�������㣨x1,y1��(x2,y2) ��˵
%��ĺ����귶Χ��0-n,��������0-m
step=5;%�������󲽳�
[leftCoord1,rightCoord1]=mysurf(imread('imL.png'),imread('imR.png'),r);%�õ�����ͼ��������ĺ�������
leftsort=sortrows(leftCoord1);
rightsort=sortrows(rightCoord1);
leftCoord1x=leftsort(:,1);
leftCoord1y=leftsort(:,2);
rightCoord1x=rightsort(:,1);
rightCoord1y=rightsort(:,2);
%   figure;%����һ���µĴ���
%   hold on;
% x1,y1�����������꣬x2,y2�����������꣬step��ÿ���������Ĳ�����diff���Ӳ�����ֵ,flag1����ͼ�ı�־����flag2����ͼ�ı�־����
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
% title('��ͼƥ������');
% axis equal;
% figure;%����һ���µĴ���
%  hold on;
% for index=1:length(leftCoord1x)
%     x1=leftCoord1x(index);
%     y1=leftCoord1y(index);
%     x2=rightCoord1x(index);
%     y2=rightCoord1y(index);
%     plotSquare( x2,y2,r );
%     [flag1,flag2,imgn1] = window1( left,right,x1,y1,x2,y2,r,yuzhi,flag1,flag2,n,m,imgn1);
% end
% title('��ͼƥ������');
% axis equal;
figure;%����һ���µĴ���
 axis ij ;
imshow(imgn,[]);
title('���Ӳ�ͼ');
% figure;%����һ���µĴ���
% imshow(imgn1,[]);
% title('���Ӳ�ͼ');


