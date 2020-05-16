 
%%
%绝对误差和算法（SAD）
%%
src=imread('Fig_rose.tif');
[a ,b ,d]=size(src);%行数（垂直分辨率）、列数（水平分辨率）、层数
 
if d==3
    src=rgb2gray(src);
end
 
mask=imread('mask.tif');
 
[m, n, d]=size(mask);
 
if d==3
    mask=rgb2gray(mask);
end
 
%%
N=n;%模板尺寸，默认模板为正方形
 
M=a;%代搜索图像尺寸，默认搜索图像为正方形
 
%%
dst=zeros(M-N+1,M-N+1);
for i=1:M-N+1         %子图选取，每次滑动一个像素
 
    for j=1:M-N+1
        
        temp=src(i:i+N-1,j:j+N-1);%当前子图
 
        dst(i,j)=sum(sum(abs(double(temp)-double(mask))));
    end
end
 
 
abs_min=min(min(dst));
 
[X,Y]=find(dst==abs_min);%最小值的位置
 
figure;
 
imshow(mask);title('模板');
 
figure;
 
imshow(src);
 
hold on;
rectangle('position',[Y,X,N,N],'edgecolor','r');%从点x，y开始绘制长宽为N-1的正方形
 
hold off;title('搜索图');
 
%%
%imwrite(uint8(src(200:260,440:500)),'mask.tif');
