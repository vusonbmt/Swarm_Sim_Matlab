function [x2,y2,heading2 ] = move_2( x1,y1,heading,ang,d)
%ǰ����Ϊλ����Ϣ��angΪ�ϰ�����������ļнǣ�delta_ang��d��������ƫת�Ǻ;���
%   delta_ang����30��
heading2=heading+ang;
 x2=x1+d*cos(heading2);
 y2=y1+d*sin(heading2);
end


