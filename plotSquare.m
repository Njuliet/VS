
%�����α߳�����A ����λ�þ���X
function [] = plotSquare( x,y,r )

hold on;
plot([x-r/2,y-r/2],[x-r/2,y+r/2]);
plot([x-r/2,y+r/2],[x+r/2,y+r/2]);
plot([x+r/2,y+r/2],[x+r/2,y-r/2]);
plot([x+r/2,y-r/2],[x-r/2,y-r/2]);
end