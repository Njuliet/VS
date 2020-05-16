function [flag1,flag2,r] = window( x1,y1,x2,y2,r,step,yuzhi,flag1,flag2,n,m)
   if( x1 >=0 && x1<=n && y1>=0 && y1<=m && x2 >=0 && x2<=n && y2>=0 && y2<=m)
        if flag1(x1-r:x1+r,y1-r:y1+r) ==0 && flag2(x2-r:x2+r,y2-r:y2+r)==0
            temp1=left(x1-r:x1+r,y1-r:y1+r);
            temp2=right(x2-r:x2+r,y2-r:y2+r);
            diff=temp1-temp2;
            disparity=sum(abs(diff));
            if disparity<yuzhi
                flag1(x1-r:x1+r,y1-r:y1+r)=1;%若符合小于阈值条件，标志置1
                flag2(x2-r:x2+r,y2-r:y2+r)=1;
            else
                flag1(x1-r:x1+r,y1-r:y1+r)=2;%若不符合视差阈值，说明有遮挡，标志置2
                flag2(x2-r:x2+r,y2-r:y2+r)=2;
            end
             r=r+step;
        end
    end
end