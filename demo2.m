close all;
fol_num=4;        
N=5;             % 4follower and 1 leader
countmax=2000;
dt=0.1;
gama=0.65;%������֮���Ӱ�����ӣ�����������ɹ��������
beta=13;%�ϰ���Ӱ������
K0=1;
KN=0.2;
goal=[25 25];
m_count = 0;
is_arrive = 0;
% x����ٶ�m/s],y����ٶ�[m/s],x��߼��ٶ�[m/ss],y��߼��ٶ�[m/ss]]
Kinematic=[0.7;0.7;0.4;0.4];
error_distance = [0;0;0;0];
color='ybgcrkr'; %%%������ɫ���
type=[2,1,0.5,0.5,2,2];%%%�����ߵ�����
start_time = clock;
%% 1-4��Ϊfollower ���һ��Ϊleader
% A=[0 1 1 1 1;     % a(ij)
%    0 0 0 0 1;
%    0 0 0 1 1;
%    0 0 1 0 1;
%    0 0 0 0 0];
A=[0 0 0 0 1;     % a(ij)%%ֻ����ǰ������˵�Ӱ��
   1 0 0 0 1;
   0 0 0 0 1;
   0 0 1 0 1;
   0 0 0 0 0];
 %% ��ʼ�� λ��pose���ٶ�V�����ٶȿ�����control
%         init_f=[-4.5 -1.5 0;%%%[x y th]
%                 -6 -1.5 pi/4; 
%                 -4.5 -4.5 -pi/4;
%                 -6 -4.5 pi/2;
%                 -3 -3 0];   
        init_f=[0 0.5 pi/4;%%%[x y th] %%�����л� ����
                0 1 pi/4; 
                0 -0.5 pi/4;
                0 -1 pi/4;
                0 0 pi/4];  
    pose_x=init_f(:,1);
    pose_y=init_f(:,2);
    pose_th=init_f(:,3);
%     ob_temp=[-10 1.2;
%              -10 2;
%              -10 12];
    %%�ϰ�������[x y]
    ob_temp=[5 4; 5 8;8 5;];
%     ob_temp=ob_temp';
    %% follower���leader��λ��
%     delta_x=[-1.5 -3 -1.5 -3 0];   % ��Լ�����   
%     delta_y=[1.5 1.5 -1.5 -1.5 0];  %�캽�����Լ������
    delta_x=[-1.5 -3 0 0 0];   % ��Լ�����   
    delta_y=[0 0 -1.5 -3 0];  %�캽�����Լ������
    V_x(:,1)=[0;0;0;0;0];
    V_y(:,1)=[0;0;0;0;0]; %%%leader��y����ĳ�ʼ�ٶ�Ϊ1m/s
    k=0;
    d_max=2;
    detect_R=1;
    ideal_posex=init_f(:,1);
    ideal_posey=init_f(:,2);
    %% ��ʼѭ�� ��˳ʱ��Բ��
    for count=1:countmax
        if count == 415  %�����л�
           delta_x=[-1 -3 -2 -4 0];   % ��Լ�����   
           delta_y=[-1 -3 -2 -4 0];  %�캽�����Լ������
        end
        if count == 620  %�����л�
           delta_x=[-1.5 -3 0 0 0];   % ��Լ�����   
           delta_y=[0 0 -1.5 -3 0];  %�캽�����Լ������
        end
        k=k+1;
%         %%%��ֱ��
%         V_x(N,k+1)=V_x(N,k);
%         V_y(N,k+1)=V_y(N,k);
%         %%%��Բ��
%         V_x(N,k+1)=cos(k*dt);
%         V_y(N,k+1)=sin(k*dt);
        %%%��Ŀ����˶�
        distance=sqrt((goal(1)-pose_x(N,k))^2+(goal(2)-pose_y(N,k))^2);%�쵼�߾���Ŀ���ľ���
        th=atan2(goal(2)-pose_y(N,k),goal(1)-pose_x(N,k));%�쵼����Ŀ���֮��ĽǶȲ�
        if distance>2   %������������Ϊ2
            distance=2;
        end
%         if distance<0.3   %������������Ϊ2
%             distance=0.3;
%         end
        V_x(N,k+1)=KN*distance*cos(th); %%����x,y������ٶ�
        V_y(N,k+1)=KN*distance*sin(th);
        mse_leader=0;
        if(rem(k,5)==1&&k>1)    %%��ʱ��֪��rem��ʲô��û��
            ideal_posex(N,(k-1)/5+1)=V_x(N,k+1)*dt*5+pose_x(N,k);
            ideal_posey(N,(k-1)/5+1)=V_y(N,k+1)*dt*5+pose_y(N,k);
        end
        %% �캽�߱���
        %%%���ǳ�ͻ������ϳ���
%         kk=0;
%         for j=1:N-1
%             kk=kk+1;
%             obs_pose(kk,1)=pose_x(j,k);
%             obs_pose(kk,2)=pose_y(j,k);
%         end
%         ob_pose=[obs_pose;ob_temp];
        ob_pose=ob_temp;
        repulsion=compute_repulsion([pose_x(N,k),pose_y(N,k)],ob_pose,detect_R);        
        %%%%%
        V_x(N,k+1)=V_x(N,k+1)+beta*repulsion(1);
        V_y(N,k+1)=V_y(N,k+1)+beta*repulsion(2);
        
         %%%���־ֲ���С�����ʩ������Ŷ� 
        if(distance>1&&abs(V_x(N,k+1))<=0.1&&abs(V_y(N,k+1))<=0.1)
%             V_x(N,k+1)=beta*(1+rand(1))*repulsion(1);
%             V_y(N,k+1)=beta*(1+rand(1))*repulsion(2);
            V_x(N,k+1)=-1+2*rand(1);
            V_y(N,k+1)=-1+2*rand(1);
        end
        %%�������˶�
        for i=1:fol_num  %fol_num=4      
            sum_delta_x=0;
            sum_delta_y=0;
            for j=1:N %%�����ھӶ�����Ӱ��
                sum_delta_x=sum_delta_x+A(i,j)*((pose_x(j,k)-pose_x(i,k))-(delta_x(j)-delta_x(i)));
                sum_delta_y=sum_delta_y+A(i,j)*((pose_y(j,k)-pose_y(i,k))-(delta_y(j)-delta_y(i)));   
            end
%             distance=[];
            error_distance(i,k+1)=sqrt(sum_delta_x^2+ sum_delta_y^2);
            th=atan2(sum_delta_y, sum_delta_x);
%             if error_distance(i,k+1)>d_max
%                 error_distance(i,k+1)=d_max;
%             end
            V_x(i,k+1)=gama*error_distance(i,k+1)*cos(th);
            V_y(i,k+1)=gama*error_distance(i,k+1)*sin(th);
%             disp(['i is',num2str(i)]);%��ӡdistance
%             disp(['distance is',num2str(distance(i,k+1))]);%��ӡdistance
%             disp(['V_x1 is',num2str(V_x(1,k+1))]);
%             disp(['V_y1 is',num2str(V_y(1,k+1))]);
            if(rem(k,5)==1&&k>1)
                ideal_posex(i,(k-1)/5+1)=V_x(i,k+1)*dt*5+pose_x(i,k);
                ideal_posey(i,(k-1)/5+1)=V_y(i,k+1)*dt*5+pose_y(i,k);
            end
           %%%���ǳ�ͻ������ϳ���
            kk=0;
            for j=1:N
                if j~=i
                    kk=kk+1;
                    obs_pose(kk,1)=pose_x(j,k);
                    obs_pose(kk,2)=pose_y(j,k);
                end
            end
            ob_pose=[obs_pose;ob_temp];
            repulsion=compute_repulsion([pose_x(i,k),pose_y(i,k)],ob_pose,detect_R);        
            %%%%%
            V_x(i,k+1)=K0*V_x(N,k)+V_x(i,k+1)+beta*repulsion(1);
            V_y(i,k+1)=K0*V_y(N,k)+V_y(i,k+1)+beta*repulsion(2);
            %%%�����ų��־ֲ���С�����ʩ������Ŷ� 
            if(error_distance(i,k+1)>0.5&&abs(V_x(i,k+1))<=0.1&&abs(V_y(i,k+1))<=0.1&&distance>1)
                V_x(i,k+1)=-1+2*rand(1);
                V_y(i,k+1)=-1+2*rand(1);
                disp(['distance is',num2str(error_distance(i,k+1))]);%��ӡdistance
                disp(['rand V_x is',num2str(V_x(i,k+1))]);
                disp(['rand V_y is',num2str(V_y(i,k+1))]);
            end
% %             out=confine([V_x(i,k) V_y(i,k)],[V_x(i,k+1) V_y(i,k+1)],Kinematic);
% %             V_x(i,k+1)=out(1);
% %             V_y(i,k+1)=out(2);
        end
        %%
        for i=1:N
            out=confine([V_x(i,k) V_y(i,k)],[V_x(i,k+1) V_y(i,k+1)],Kinematic,0.1);
%             out=[V_x(i,k+1) V_y(i,k+1)];
            V_x(i,k+1)=out(1);
            V_y(i,k+1)=out(2);
            pose_x(i,k+1)=pose_x(i,k)+dt*V_x(i,k+1);
            pose_y(i,k+1)=pose_y(i,k)+dt*V_y(i,k+1);
            pose_th(i,k+1)=atan2(V_y(i,k+1),V_x(i,k+1));
        end
        tt_x(1:4,k)=pose_x(5,k);
        error_x(:,k)=tt_x(1:4,k)-pose_x(1:4,k)+(delta_x(1:4))';
        tt_y(1:4,k)=pose_y(5,k);
        error_y(:,k)=tt_y(1:4,k)-pose_y(1:4,k)+(delta_y(1:4))';
        %% ====Animation====
        area = compute_area(pose_x(N,k+1),pose_y(N,k+1),10);
        hold off;
        ArrowLength=0.7;% ��ͷ����
        for j=1:N
            quiver(pose_x(j,k+1),pose_y(j,k+1),ArrowLength*cos(pose_th(j,k+1)),ArrowLength*sin(pose_th(j,k+1)),'.','color',color(1,j),'LineWidth',1.3);hold on;
            draw_circle(pose_x(j,k+1),pose_y(j,k+1),0.1,j);hold on;
        end
        obn = size(ob_temp);
        for i =1:obn
            draw_square(ob_temp(i,1),ob_temp(i,2),0.2);hold on;
        end
        xlabel('x Position(m)');
        ylabel('y Position(m)');
%         plot(ob_temp(:,1),ob_temp(:,2),'^k','LineWidth',2);hold on;
        x1 = [3,7,6,2];
        y1 = [5,9,10,6];
        x2 = [5,9,10,6];
        y2 = [3,7,6,2];
        x1=x1+8;y1=y1+8;x2=x2+8;y2=y2+8;
        fill(x1,y1,'k')             % �����ͼ���������Ϊ��ɫ
        fill(x2,y2,'k')             % �����ͼ���������Ϊ��ɫ
%         area=[-10 10 -10 10];
        axis(area);
        grid on;
        drawnow;    
        %% �ж���ֹ����
        now=[pose_x(N,k+1),pose_y(N,k+1)];
        if norm(now-goal)<0.2
            is_arrive = 1;
            m_count = m_count + 1;
            %�����л� פͣ
            delta_x=[-1 -2 1 2 0];   % ��Լ�����   
            delta_y=[0 0 0 0 0];  %�캽�����Լ������
        end
        if m_count > 100
           end_time = clock;
           disp('Arrive Goal!!');break;
        end
    end
    %% ��ͼ
    figure                               
    for i=1:N  %%·��
        plot(pose_x(i,:),pose_y(i,:),color(1,i),'LineWidth',1.5);
        hold on
    end
    for i=1:N  %%����������·���л�����λ��
        plot(pose_x(i,1),pose_y(i,1),'p','color',color(1,i),'LineWidth',2);
        hold on
        draw_circle(pose_x(i,300),pose_y(i,300),0.2,i);hold on;
        draw_circle(pose_x(i,570),pose_y(i,570),0.2,i);hold on;
        draw_circle(pose_x(i,760),pose_y(i,760),0.2,i);hold on;
    end
%     plot(pose_x(N,1),pose_y(N,1),'p','color',color(1,N),'LineWidth',2);
%     hold on
    for i=1:N  %%�������յ�
        plot(pose_x(i,k),pose_y(i,k),'h','color',color(1,i),'LineWidth',2);
        hold on
    end
%     plot(pose_x(N,k),pose_y(N,k),'h','color',color(1,N),'LineWidth',2);
%     hold on
    for i =1:obn
        draw_square(ob_temp(i,1),ob_temp(i,2),0.2);hold on;
    end
    %plot(ob_temp(:,1),ob_temp(:,2),'Xk','LineWidth',2);hold on;
    grid on;
    fill(x1,y1,'k')             % �����ͼ���������Ϊ��ɫ
    fill(x2,y2,'k')             % �����ͼ���������Ϊ��ɫ
    xlabel('x');
    ylabel('y');
    legend('follower1','follower2','follower3','follower4','leader','Location','NorthWest');
    xlabel('x Position(m)');
    ylabel('y Position(m)');
    title('��������ͼ������쵼�߷���������˱�ӿ����㷨');
    %% �����ͼ
    cost_time = 3600*(end_time(4)-start_time(4)) + 60 * (end_time(5)-start_time(5)) + (end_time(6) - start_time(6));
    kx=cost_time/k;
    cx=0:kx:cost_time;
    figure                                %   ������άƽ��ͼ  ����
    error=sqrt(error_x.^2+error_y.^2);
    for i=1:4
        plot(cx(1:k-1),error(i,1:k-1),color(1,i),'LineWidth',1.5);
        hold on;
    end
    legend('follower1','follower2','follower3','follower4');
    xlabel('ʱ��(s)');
    ylabel('λ�����(m)');
    title('������˱�ӿ��Ƹ������˷����������');
    
function [ next] = confine(current,next,Kinematic,dt)
%%%current=[v_x v_y];
%%%%Kinematic=[ x����ٶ�m/s],y����ٶ�[m/s],x��߼��ٶ�[m/ss],y��߼��ٶ�[m/ss]]
%%%Kinematic=[1;1;0.5;0.5];
%% �ٶ�x�ϵ�����
delta_x=next(1)-current(1);
if delta_x>=0
    next(1)=min(current(1)+delta_x,current(1)+Kinematic(3)*dt);
else
    next(1)=max(current(1)+delta_x,current(1)-Kinematic(3)*dt);
end
if next(1)>=0
    next(1)=min(next(1),Kinematic(1));
else
    next(1)=max(next(1),-Kinematic(1));
end
%% �ٶ�y�ϵ�����
delta_y=next(2)-current(2);
if delta_y>=0
    next(2)=min(current(2)+delta_y,current(2)+Kinematic(4)*dt);
else
    next(2)=max(current(2)+delta_y,current(2)-Kinematic(4)*dt);
end
if next(2)>=0
    next(2)=min(next(2),Kinematic(2));
else
    next(2)=max(next(2),-Kinematic(2));
end
end

