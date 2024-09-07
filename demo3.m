close all;
fol_num=4;
N=5;             % 4follower and 1 leader
countmax=2000;
dt=0.1;
gama=0.65;%Influence factor between robots. Too large may cause overshoot and vibration.
beta=13;%Influence factor of obstacles
K0=1;
KN=0.2;
goal=[25 25];
m_count = 0;
is_arrive = 0;
% x maximum speed [m/s], y maximum rotation speed [rad/s], x maximum acceleration [m/ss], y maximum acceleration [rad/ss]]
Kinematic=[0.7;0.7;0.4;0.4];
attmse(:,1) = [0;0;0;0;0;0];
error_distance = [0;0;0;0];
color='ybgcrkr'; %%%Define the color marker
type=[2,1,0.5,0.5,2,2]; %%%Define the line type
start_time = clock;
%% 1-4 lines are followers and the last line is a leader
A=[0 0 0 0 1;     % a(ij)%% only considers the influence of the front robot
    1 0 0 0 1;
    0 0 0 0 1;
    0 0 1 0 1;
    0 0 0 0 0];
%% Initialize position pose, velocity V, acceleration control
init_f=[-1.5 0 pi/4;%%%[x y th] %%Formation switching start
    -3 0 pi/4;
    0 -1.5 pi/4;
    0 -3 pi/4;
    0 0 pi/4];
pose_x=init_f(:,1);
pose_y=init_f(:,2);
pose_th=init_f(:,3);

%%Obstacle coordinates [x y]
ob_temp=[5 4; 5 8;8 5;];
%     ob_temp=ob_temp';
%%Follower's position relative to leader
delta_x=[-1.5 -3 0 0 0]; % Relative interval error
delta_y=[0 0 -1.5 -3 0]; % There is no error between the leader and itself
V_x(:,1)=[0;0;0;0;0];
V_y(:,1)=[0;0;0;0;0]; %%% The initial speed of the leader in the y direction is 1m/s
k=0;
d_max=2;
detect_R=1;
ideal_posex=init_f(:,1);
ideal_posey=init_f(:,2);
%% Start the cycle and go in a clockwise circle
for count=1:countmax
    if count == 415 % formation switching
        delta_x=[-1 -3 -2 -4 0]; % relative interval error
        delta_y=[-1 -3 -2 -4 0]; % no error between the navigator and itself
    end
    if count == 620 % formation switching
        delta_x=[-1.5 -3 0 0 0]; % relative interval error
        delta_y=[0 0 -1.5 -3 0]; % no error between the navigator and itself
    end
    k=k+1;
    %%%Move towards the target point
    distance=sqrt((goal(1)-pose_x(N,k))^2+(goal(2)-pose_y(N,k))^2);%The distance between the leader and the target point
    th=atan2(goal(2)-pose_y(N,k),goal(1)-pose_x(N,k));%The angle difference between the leader and the target point
    if distance>2   %将最大距离设置为2
        distance=2;
    end
    V_x(N,k+1)=KN*distance*cos(th); %%设置x,y方向的速度
    V_y(N,k+1)=KN*distance*sin(th);
    mse_leader=0;
    if(rem(k,5)==1&&k>1)    %%暂时不知道rem是什么，没用
        ideal_posex(N,(k-1)/5+1)=V_x(N,k+1)*dt*5+pose_x(N,k);
        ideal_posey(N,(k-1)/5+1)=V_y(N,k+1)*dt*5+pose_y(N,k);
    end
    %% 领航者避障
    %%%考虑冲突避免加上斥力
    ob_pose=ob_temp;
    repulsion=compute_repulsion([pose_x(N,k),pose_y(N,k)],ob_pose,detect_R);
    %%%%%
    V_x(N,k+1)=V_x(N,k+1)+beta*repulsion(1);
    V_y(N,k+1)=V_y(N,k+1)+beta*repulsion(2);
    
    %%%出现局部极小的情况施加随机扰动
    if(distance>1&&abs(V_x(N,k+1))<=0.1&&abs(V_y(N,k+1))<=0.1)
        V_x(N,k+1)=-1+2*rand(1);
        V_y(N,k+1)=-1+2*rand(1);
    end
    att_mse=[]; %%暂时不知道用来做什么，没用
    if(rem(k,5)==1&&k>1)
        attmse(N+1,(k-1)/5)=0;
        for j=1:fol_num
            att_mse(j) = cal_mse([pose_x(j,k),pose_y(j,k)],[ideal_posex(j,(k-1)/5),ideal_posey(j,(k-1)/5)]);
            attmse(j,(k-1)/5) = abs(att_mse(j)-0.2);
            attmse(N+1,(k-1)/5) = attmse(N+1,(k-1)/5) + abs(att_mse(j)-0.2);
        end
    end
    %%跟随者运动
    for i=1:fol_num  %fol_num=4
        sum_delta_x=0;
        sum_delta_y=0;
        for j=1:N %%考虑邻居对它的影响
            sum_delta_x=sum_delta_x+A(i,j)*((pose_x(j,k)-pose_x(i,k))-(delta_x(j)-delta_x(i)));
            sum_delta_y=sum_delta_y+A(i,j)*((pose_y(j,k)-pose_y(i,k))-(delta_y(j)-delta_y(i)));
        end

        error_distance(i,k+1)=sqrt(sum_delta_x^2+ sum_delta_y^2);
        th=atan2(sum_delta_y, sum_delta_x);

        V_x(i,k+1)=gama*error_distance(i,k+1)*cos(th);
        V_y(i,k+1)=gama*error_distance(i,k+1)*sin(th);

        if(rem(k,5)==1&&k>1)
            ideal_posex(i,(k-1)/5+1)=V_x(i,k+1)*dt*5+pose_x(i,k);
            ideal_posey(i,(k-1)/5+1)=V_y(i,k+1)*dt*5+pose_y(i,k);
        end
        %%%考虑冲突避免加上斥力
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
        %%%跟随着出现局部极小的情况施加随机扰动
        if(error_distance(i,k+1)>0.5&&abs(V_x(i,k+1))<=0.1&&abs(V_y(i,k+1))<=0.1&&distance>1)
            V_x(i,k+1)=-1+2*rand(1);
            V_y(i,k+1)=-1+2*rand(1);
            disp(['distance is',num2str(error_distance(i,k+1))]);%打印distance
            disp(['rand V_x is',num2str(V_x(i,k+1))]);
            disp(['rand V_y is',num2str(V_y(i,k+1))]);
        end

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
    if(rem(k,5)==1&&k>1)
        mse_leader = cal_mse([pose_x(N,k),pose_y(N,k)],[ideal_posex(N,(k-1)/5),ideal_posey(N,(k-1)/5)]);
        attmse(N,(k-1)/5)=mse_leader;
    end
    tt_x(1:4,k)=pose_x(5,k);
    error_x(:,k)=tt_x(1:4,k)-pose_x(1:4,k)+(delta_x(1:4))';
    tt_y(1:4,k)=pose_y(5,k);
    error_y(:,k)=tt_y(1:4,k)-pose_y(1:4,k)+(delta_y(1:4))';
    if(k==100)
        bbb=1;
    end
    %% ====Animation====
    area = compute_area(pose_x(N,k+1),pose_y(N,k+1),10);
    hold off;
    ArrowLength=0.7;%
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
    %         plot(ob_temp(:,1),ob_temp(:,2),'Xk','LineWidth',2);hold on;
    x1 = [3,7,6,2];
    y1 = [5,9,10,6];
    x2 = [5,9,10,6];
    y2 = [3,7,6,2];
    x1=x1+8;y1=y1+8;x2=x2+8;y2=y2+8;
    fill(x1,y1,'k')             % 画填充图，填充区域为绿色
    fill(x2,y2,'k')             % 画填充图，填充区域为绿色
    %         area=[-10 10 -10 10];
    axis(area);
    grid on;
    drawnow;
    %% 判断终止条件
    now=[pose_x(N,k+1),pose_y(N,k+1)];
    if norm(now-goal)<0.2
        is_arrive = 1;
        end_time = clock;
        disp('Arrive Goal!!');break;
    end
end
attmse(:,100)=[0;0;0;0;0;0];
for i=1:5
    dmax(i)=max(attmse(i,1:99));
end
for i=1:5
    if(dmax(i)>0.1)%排除一些噪声进行逐行归一化
        attmse(i,1:99)=normalization(attmse(i,1:99),0,dmax(i),0,1);
    else
        attmse(i,1:99)=normalization(attmse(i,1:99),0,max(dmax(:)),0,1);
    end
end
save('attmse.mat','attmse');

%% 画图
figure
for i=1:N
    plot(pose_x(i,:),pose_y(i,:),color(1,i),'LineWidth',2);
    hold on
end
for i=1:N
    plot(pose_x(i,1),pose_y(i,1),'p','color',color(1,i),'LineWidth',2);
    hold on
    draw_circle(pose_x(i,300),pose_y(i,300),0.2,i);hold on;
    draw_circle(pose_x(i,570),pose_y(i,570),0.2,i);hold on;
    draw_circle(pose_x(i,760),pose_y(i,760),0.2,i);hold on;
end
for i=1:N
    plot(pose_x(i,k),pose_y(i,k),'h','color',color(1,i),'LineWidth',2);
    hold on
end
for i =1:obn
    draw_square(ob_temp(i,1),ob_temp(i,2),0.2);hold on;
end
%     plot(ob_temp(:,1),ob_temp(:,2),'Xk','LineWidth',2);hold on;
grid on;
fill(x1,y1,'k')             % 画填充图，填充区域为绿色
fill(x2,y2,'k')             % 画填充图，填充区域为绿色
xlabel('x');
ylabel('y');
legend('follower1','follower2','follower3','follower4','leader','Location','NorthWest');
xlabel('x Position(m)');
ylabel('y Position(m)');
%     title('First-order formation algorithm based on consistency');
title('Obstacle avoidance and formation switching control algorithm');
%% 画误差图
cost_time = 3600*(end_time(4)-start_time(4)) + 60 * (end_time(5)-start_time(5)) + (end_time(6) - start_time(6));
kx=cost_time/k;
cx=0:kx:cost_time;
figure                                %   生成三维平面图  连续
error=sqrt(error_x.^2+error_y.^2);
for i=1:4
    plot(cx(1:k-1),error(i,1:k-1),color(1,i),'LineWidth',1.5);
    hold on;
end
legend('follower1','follower2','follower3','follower4');
xlabel('time(s)');
ylabel('Position error(m)');
title('Simulation error curve of each robot in the five-robot formation obstacle avoidance and formation switching');

function [ next] = confine(current,next,Kinematic,dt)
%%%current=[v_x v_y];
%%%%Kinematic=[ x maximum speed [m/s], y maximum speed [m/s], x maximum acceleration [m/ss], y maximum acceleration [m/ss]]
%%%Kinematic=[1;1;0.5;0.5];
%% Speed ??limit x
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
%% Speed ??limit on y
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

function [ mse ] = cal_mse(pose,ideal_pose)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
mse = 0;
mse = ((pose(1)-ideal_pose(1))^(2)+(pose(2)-ideal_pose(2))^(2))^(1/2);

end

function [ area ] = compute_area(x,y,range)
%UNTITLED4 A summary of this function is shown here
% detailed instructions are shown here
if(nargin==2)
    range=10;
end

if(x>=0)
    num_x=floor(x(1)/range);
    x1=num_x*range;
    x2=(num_x+1)*range;
else
    num_x=floor(x(1)/-range);
    x1=-range*(num_x+1);
    x2=-range*num_x;
end
if(y>=0)
    num_y=floor(y(1)/range);
    y1=num_y*range;
    y2=(num_y+1)*range;
else
    num_y=floor(y(1)/-range);
    y1=-range*(num_y+1);
    y2=-range*num_y;
end
% area=[x1-3 x2+3 y1-3 y2+3];
area=[-5 30 -5 30];
end

function [ repulsion] = compute_repulsion(robot_pose,obs_pose,detect_R)
% Collision avoidance The detection range does not take into account the angle
% obs_pose=[x1 y1;x2; y2;....]
[M,N]=size(obs_pose);
repulsion(1)=0; %x-direction repulsion
repulsion(2)=0; %y-direction repulsion
for i=1:M
    distance=sqrt((robot_pose(1)-obs_pose(i,1))^2+(robot_pose(2)-obs_pose(i,2))^2);
    if distance<=detect_R
        temp=1.0*(1/distance-1/detect_R)/(distance^3)*(distance^5);
        repulsion(1)=repulsion(1)+temp*(robot_pose(1)-obs_pose(i,1));
        repulsion(2)=repulsion(2)+temp*(robot_pose(2)-obs_pose(i,2));
    end
end
end

function [ output_args ] = draw_circle (x,y,r,n)
%UNTITLED4 Summary of this function is shown here
% Detailed description is shown here
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

function [ output_args ] = draw_circle2 (x,y,r,color)
%UNTITLED4 Summary of this function is shown here
% Detailed description is shown here
% color='-r';
t=0:0.01:2*pi;
X=x+r*cos(t);
Y=y+r*sin(t);
plot(X,Y,color,'LineWidth',1);
end

function [ output_args ] = draw_secor( x,y,th1,th2,r )
%UNTITLED3 A summary of this function is shown here
% is shown here in detail th is the radian value
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

function [ output_args ] = draw_square (x,y,r)
x1 = [x-r,x+r,x+r,x-r];
y1 = [y+r,y+r,y-r,y-r];
fill(x1,y1,'k')
end

function interpoint( x1,y1,x2,y2,x3,y3,colo,lstyle)

if (nargin==6)
    colo='k';
    lstyle=':';
end
syms k b m n x y;
if(x1==x2)%x1x2 line slope does not exist
    solx=x1;
    soly=y3;
elseif(y1==y2)% The slope of the x1x2 line is 0
    solx=x3;
    soly=y1;
else 
    solk=(y2-y1)/(x2-x1);
    solb=y2-solk*x2;
    solk1=-1/solk;
    solb1=y3-solk1*x3;
    solx=(solb1-solb)/(solk-solk1);
    soly=solk*solx+solb;
%     [solx,soly] = solve(solk1*x-y+solb1==0,solk*x-y+solb==0,x,y);
end
line([x1,solx],[y1,soly],'color',colo,'linestyle',lstyle);
line([x3,solx],[y3,soly],'color',colo,'linestyle',lstyle);
end

function x = motion(x, u,dt)
% Motion Model
% x=[x;y;th]  u = [vt; wt];%%Current speed and angular velocity

delta_th=u(2)*dt;
delta_x=u(1)*dt*cos(x(3)+delta_th/2);
delta_y=u(1)*dt*sin(x(3)+delta_th/2);
x(1)=x(1)+delta_x;
x(2)=x(2)+delta_y;
x(3)=x(3)+delta_th;

end

function [x2,y2,heading2 ] = move_2( x1,y1,heading,ang,d)
%The first three items are position information, ang is the angle between the obstacle and the central axis, delta_ang and d are used to determine the deflection angle and distance
% delta_ang can be set to 30 degrees
heading2=heading+ang;
 x2=x1+d*cos(heading2);
 y2=y1+d*sin(heading2);
end

function [ feature_new ] = normalization( feature ,mi,ma,lower,upper)
%UNTITLED Show summary about this function here
% Show detailed description here
feature_new=lower+(upper-lower)*(feature-mi)/(ma-mi);
end

function [ result ] = seek_ang( th1,th2 )
%UNTITLED4 radians
% th1 is the angle to be rotated, th2 is the angle of the current position
% th1-th2 is the angle of the main axis relative to the reference point

result=th1-th2;
if(th1>0)
    if(result>pi)
        result=result-2*pi;
    end
else
    if(result<-pi)
        result=result+2*pi;
    end
end

end


function degree = toDegree(radian)
% radian to degree
degree = radian/pi*180;
end

function radian = toRadian(degree)
% degree to radian
radian = degree/180*pi;

end