function [ output_args ] = draw_circle (x,y,r,n)
%UNTITLED4 �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
% if(nargin==4)
%     color='-k';
% end
color='ybgcrkr';
% if (n == 5)
%     color ='r';
% else
%     color ='-k';
% end
t=0:0.01:2*pi;
X=x+r*cos(t);
Y=y+r*sin(t);
plot(X,Y,color(1,n),'LineWidth',3);
end

