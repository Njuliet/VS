clc;
clear;
left=double(rgb2gray(imread('imL.png')));
right=double(rgb2gray(imread('imR.png')));

[m ,n]=size(left); %��������ֱ�ֱ��ʣ���������ˮƽ�ֱ��ʣ�

r=3;       %��ʼ���ڱ߳���һ��
imgn=zeros(m,n); 

flag1=zeros(m,n);  %������־��ͼ�е����ص��Ƿ���ʹ�  ͼ������ϵ����ͼ������Ͻ�Ϊԭ��
flag2=zeros(m,n);  %��ͼ   ���ֱ߽��������
% flag1=zeros(400,500);  %������־��ͼ�е����ص��Ƿ���ʹ�  ͼ������ϵ����ͼ������Ͻ�Ϊԭ��
% flag2=zeros(400,500);

yuzhi=20;
%��ÿһ�������㣨x1,y1��(x2,y2) ��˵
%��ĺ����귶Χ��0-n,��������0-m
step=5;%�������󲽳�
[leftCoord1x,leftCoord1y,rightCoord1x,rightCoord1y]=mysurf(imread('imL.png'),imread('imR.png'),r);%�õ�����ͼ��������ĺ�������
 figure;%����һ���µĴ���
 hold on;
% x1,y1�����������꣬x2,y2�����������꣬step��ÿ���������Ĳ�����diff���Ӳ�����ֵ,flag1����ͼ�ı�־����flag2����ͼ�ı�־����
for index=1:length(leftCoord1x)
    x1=leftCoord1x(index);
    y1=leftCoord1y(index);
    x2=rightCoord1x(index);
    y2=rightCoord1y(index);
    plotSquare( x1,y1,r );
    [flag1,flag2,imgn] = window( left,right,x1,y1,x2,y2,r,yuzhi,flag1,flag2,n,m,imgn);
end
axis equal;
figure;%����һ���µĴ���
 hold on;
for index=1:length(leftCoord1x)
    x1=leftCoord1x(index);
    y1=leftCoord1y(index);
    x2=rightCoord1x(index);
    y2=rightCoord1y(index);
    plotSquare( x2,y2,r );
    [flag1,flag2,imgn] = window( x1,y1,x2,y2,r,yuzhi,flag1,flag2,n,m,imgn);
end
axis equal;
imshow(imgn,[]);



