A=[1,3,5,7;2,4,6,8];%����һ����ά����,2�У�4��
B=A';%ת��
Y=pdist(B);%�������̫���˿���ֱ��������������е���������룬��ǰ������������ߵ�ľ��룬���κ���

a=[12 35 24;26 78 14;16 19 21];
%������е����ֵ����Сֵmax(max(a))   min(min(a))
x=3;
y=4;
r=1;
plot1(3,4,1);%д�˸�������plot1
%rectangle('Position',[x-r,y-r,2*r,2*r],'Curvature',[1,1],'linewidth',1);
% axis equal

% x=size(sceneImageL);%��ȡͼ��Ĵ�С
% min_x=0;
% min_y=0;
% max_x=x(:,1);%��ȡͼ��Ŀ�
% max_y=x(:,2);%��ȡͼ��ĳ�
% imagesc([min_x max_x], [min_y max_y], flip(sceneImageL,1));
% hold on
%  plot1(a2,a3,mindistance);