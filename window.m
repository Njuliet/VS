function [flag1,flag2,imgn] = window( left,right,x1,y1,x2,y2,r,yuzhi,flag1,flag2,n,m,imgn)
   if( round(x1)-r >=0 && round(x1)+r<=m && round(y1)-r>=0 && round(y1)+r<=n && round(x2)-r >=0 && round(x2)+r<=m && round(y2)-r>=0 && round(y2)+r<=n)
       sum1=sum(sum(flag1(round(x1)-r:round(x1)+r,round(y1)-r:round(y1)+r)));
       sum2=sum(sum(flag2(round(x2)-r:round(x2)+r,round(y2)-r:round(y2)+r)));
       diff=zeros(2*r+1,2*r+1);
        if sum1 ==0 && sum2==0  %原始正方形 ,这个正方形的边长一定是奇数，因为是以中心点向左三步向右三步一共奇数步
            temp1=left(round(x1)-r:round(x1)+r,round(y1)-r:round(y1)+r);
            temp2=right(round(x2)-r:round(x2)+r,round(y2)-r:round(y2)+r);
            diff=temp1-temp2;
            disparity=sum(sum(abs(diff)));
            if disparity<yuzhi
                flag1(round(x1)-r:round(x1)+r,round(y1)-r:round(y1)+r)=1;%若符合小于阈值条件，标志置1
                flag2(round(x2)-r:round(x2)+r,round(y2)-r:round(y2)+r)=1;
            else
                flag1(round(x1)-r:round(x1)+r,round(y1)-r:round(y1)+r)=2;%若不符合视差阈值，说明有遮挡，标志置2
                flag2(round(x2)-r:round(x2)+r,round(y2)-r:round(y2)+r)=2;
            end
        else
            %扩大边长后的正方形
        end
          imgn(round(x1)-r:round(x1)+r,round(y1)-r:round(y1)+r)=diff;
    end
end