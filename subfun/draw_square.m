function [ output_args ] = draw_square (x,y,r)
%UNTITLED4 �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
% if(nargin==4)
%     color='-k';
% end
% if (n == 5)
%     color ='r';
% else
%     color ='-k';
% end
x1 = [x-r,x+r,x+r,x-r];
y1 = [y+r,y+r,y-r,y-r];
fill(x1,y1,'k')
end

