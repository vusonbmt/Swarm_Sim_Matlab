function [ output_args ] = draw_secor( x,y,th1,th2,r )
%UNTITLED3 �˴���ʾ�йش˺�����ժҪ
%   �˴���ʾ��ϸ thΪ����ֵ
t=th1:0.01:th2;
X=x+r*cos(t);
Y=y+r*sin(t);
plot(X,Y,'LineWidth',1);
X=[x,x+r*cos(th1)];
Y=[y,y+r*sin(th1)];
line(X,Y,'LineWidth',1);
X=[x,x+r*cos(th2)];
Y=[y,y+r*sin(th2)];
line(X,Y,'LineWidth',1);
end

