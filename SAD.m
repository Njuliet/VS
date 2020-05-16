 
%%
%���������㷨��SAD��
%%
src=imread('Fig_rose.tif');
[a ,b ,d]=size(src);%��������ֱ�ֱ��ʣ���������ˮƽ�ֱ��ʣ�������
 
if d==3
    src=rgb2gray(src);
end
 
mask=imread('mask.tif');
 
[m, n, d]=size(mask);
 
if d==3
    mask=rgb2gray(mask);
end
 
%%
N=n;%ģ��ߴ磬Ĭ��ģ��Ϊ������
 
M=a;%������ͼ��ߴ磬Ĭ������ͼ��Ϊ������
 
%%
dst=zeros(M-N+1,M-N+1);
for i=1:M-N+1         %��ͼѡȡ��ÿ�λ���һ������
 
    for j=1:M-N+1
        
        temp=src(i:i+N-1,j:j+N-1);%��ǰ��ͼ
 
        dst(i,j)=sum(sum(abs(double(temp)-double(mask))));
    end
end
 
 
abs_min=min(min(dst));
 
[X,Y]=find(dst==abs_min);%��Сֵ��λ��
 
figure;
 
imshow(mask);title('ģ��');
 
figure;
 
imshow(src);
 
hold on;
rectangle('position',[Y,X,N,N],'edgecolor','r');%�ӵ�x��y��ʼ���Ƴ���ΪN-1��������
 
hold off;title('����ͼ');
 
%%
%imwrite(uint8(src(200:260,440:500)),'mask.tif');
