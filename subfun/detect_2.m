function [ in,ang,r] = detect_2( x1,y1,x2,y2,th,threld_th,radius )
%thΪ��1�ķ�λ��
%threld_th,radius�ֱ�����ɨ��Ǻ�ɨ��뾶
%angΪ�ϰ����������ļн�
%%�ж�һ�����Ƿ�����ɨ��������
r=sqrt((x2-x1)^2+(y2-y1)^2);
t=atan2(y2-y1,x2-x1); %%%����ֵ
%%%��ǶȲ�
ang= seek_ang(t,th);
if(r<=radius)
    %%ang=angle(x1,y1,x2,y2)-th;
    if(abs(ang)<=threld_th)
        in=1;
    else
        in=0;
    end
else
    in=0;
end



