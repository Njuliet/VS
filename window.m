function [flag1,flag2,imgn] = window( left,right,x1,y1,x2,y2,r,flag1,flag2,n,m,imgn)
if( round(x1)-r >=0 && round(x1)+r<=m && round(y1)-r>=0 && round(y1)+r<=n && round(x2)-r >=0 && round(x2)+r<=m && round(y2)-r>=0 && round(y2)+r<=n)
    WindowLeft=zeros(2*r+1,2*r+1);    
     b=1;
    for j=round(y1)-r:round(y1)+r
        a=1;
        for i=round(x1)-r:round(x1)+r
            if(flag1(i,j)==0)
                WindowLeft(a,b)=i;
                flag1(i,j)=1;
            end
                a=a+1;             
        end
        b=b+1;
    end
    WindowRight=zeros(2*r+1,2*r+1);
    b=1;
    for j=round(y2)-r:round(y2)+r
        a=1;
        for i=round(x2)-r:round(x2)+r
            if(flag2(i,j)==0)
                WindowRight(a,b)=i;
                flag2(i,j)=1;
            end
               a=a+1;           
        end
        b=b+1;
    end
    disparity=WindowLeft-WindowRight;
    c=1;
    for j=round(y1)-r:round(y1)+r
        d=1;
        for i=round(x1)-r:round(x1)+r
            if imgn(i,j)==0
                imgn(i,j)=disparity(d,c);
            end
            d=d+1;
        end
        c=c+1;
    end
    %           imgn(round(x1)-r:round(x1)+r,round(y1)-r:round(y1)+r)=disparity;
%     axis ij ;
%     plotSquare( x1,y1,r );
%     
%     title('²âÊÔ×óÊÓ²îÍ¼window');
%     axis equal;
end
end