
%正方形边长矩阵A 中心位置矩阵X
function [] = plotSquare( x,y,r )

hold on;
plot([x-r/2,y-r/2],[x-r/2,y+r/2]);
plot([x-r/2,y+r/2],[x+r/2,y+r/2]);
plot([x+r/2,y+r/2],[x+r/2,y-r/2]);
plot([x+r/2,y-r/2],[x-r/2,y-r/2]);
end