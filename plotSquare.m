
%�����α߳�r  ����λ��x,y
function [] = plotSquare( x,y,r )

hold on;
plot([x-r,x-r],[y-r,y+r]);
plot([x-r,x+r],[y+r,y+r]);
plot([x-r,x+r],[y-r,y-r]);
plot([x+r,x+r],[y+r,y-r]);
axis square;
end