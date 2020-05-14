A=[1,3,5,7;2,4,6,8];%创建一个二维数组,2行，4列
B=A';%转置
Y=pdist(B);%这个函数太好了可以直接求出矩阵中所有点的两两距离，以前个点求其他后边点的距离，依次后推

a=[12 35 24;26 78 14;16 19 21];
%求矩阵中的最大值和最小值max(max(a))   min(min(a))
x=3;
y=4;
r=1;
plot1(3,4,1);%写了个函数叫plot1
%rectangle('Position',[x-r,y-r,2*r,2*r],'Curvature',[1,1],'linewidth',1);
% axis equal

% x=size(sceneImageL);%获取图像的大小
% min_x=0;
% min_y=0;
% max_x=x(:,1);%获取图像的宽
% max_y=x(:,2);%获取图像的长
% imagesc([min_x max_x], [min_y max_y], flip(sceneImageL,1));
% hold on
%  plot1(a2,a3,mindistance);