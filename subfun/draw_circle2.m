function [ output_args ] = draw_circle2 (x,y,r,color)
%UNTITLED4 �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ˵��
% color='-r';
t=0:0.01:2*pi;
X=x+r*cos(t);
Y=y+r*sin(t);
plot(X,Y,color,'LineWidth',1);
end

