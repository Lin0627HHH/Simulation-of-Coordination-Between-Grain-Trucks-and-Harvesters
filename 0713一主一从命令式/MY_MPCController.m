%从机
function[sys, x0, str, ts] = MY_MPCController(t, x, u, flag)
switch flag
    case 0   %初始化
        [sys, x0, str, ts] = mdlInitializeSizes;  %初始化
    case 2  %更新离散状态
        sys = mdlUpdates(t, x, u);
    case 3  %计算输出
        sys = mdlOutputs(t, x, u);
    case {1, 4, 9, 10}  % Unused flags
        sys = [];
    otherwise  %未知的flag
        error(['unhandled flag =' ,num2str(flag)]);  %Error handling
end
% s函数主程序结束

% 初始化子函数
function[sys, x0, str, ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates = 0;  % 连续状态量的个数
sizes.NumDiscStates = 3;  % 离散状态量的个数
sizes.NumOutputs = 2;  % 输出量的个数
sizes.NumInputs = 4;  %输入量的个数
sizes.DirFeedthrough = 1;  % 矩阵D非空， 直接贯通标志
sizes.NumSampleTimes = 1;  % 采样时间的个数
sys = simsizes(sizes);
x0 = [0; 0; 0];  % 状态量初始化



global T1 L;
T1=0.1;%控制器计算周期
L=2.05;%农机车辆轴距
global Nx Nu Np Nc;
Nx=3;%状态量的个数
Nu=2;%控制量的个数
Np=5;%预测步长
Nc=5;%控制步长
global point2 count ;
count=0;%控制器求解次数记数
% point_flag=0;%期望轨迹初次计算标志
point2=1;%%期望点初始化
% x0=zeros(1,Nc*2);%%优化初始值初始化

global r ;


%%%运粮车停车的点暂定为(0,-8)
%%%设置A点和B点坐标
ax=0;
ay=-8;
bx=300;
by=-8;





%%%设置农机转弯半径
R=5;
%%%设置工作行距（需大于或等于两倍农机转弯半径）
workingdistance=11;
%%%设置农机工作和掉头行驶速度（公里/小时）
workingvelocity=10;
turningvelocity=5;
%%%设置地块宽度
breadth=100;
%%%设置路径点生成周期（秒）
global T
T=0.01;
%%%求得耕作行数
row=floor(breadth/workingdistance);
%%%设置初始转向（0为逆时针方向，1为顺时针方向）
initialturning=0;
%%%定义AB线及其坐标
abx=bx-ax;
aby=by-ay;
direction=[];
%%%确定AB线方向（从A到B）
if ax==bx
    if aby>0
        direction=pi/2;
    elseif aby<0
        direction=-pi/2;
    end
elseif ay==by
    if abx>0
        direction=0;
    elseif abx<0
        direction=pi;
    end
else
    direction=atan(aby/abx);
end
%%%确定AB线长度（直线段工作长度）
distance=sqrt(abx^2+aby^2);
global quadrant turningstraightlinepoint workingpathpoint;
%%%计算工作直线、转弯首四分之一圆、转弯直线、转弯后四分之一圆和总路径点数,并向下取整
workingpathpoint=floor(distance/((workingvelocity*1000/3600)/(1/T)));
quadrant=floor((2*pi*R/4)/((turningvelocity*1000/3600)/(1/T)));
turningstraightlinepoint=floor((workingdistance-2*R)/((turningvelocity*1000/3600)/(1/T)));
global acceleration_distance
global acceleration_distance_count
global add_velocity
interval_workingpathpoints_distance=((workingvelocity*1000/3600)/(1/T));
acceleration_distance_count=0;
acceleration_distance=5;
d=0;
for i=1:1:workingpathpoint
    d=d+interval_workingpathpoints_distance;
    if d>=acceleration_distance
        acceleration_distance_count=i;
        break;
    end
end

add_velocity=(workingvelocity*1000/3600-turningvelocity*1000/3600)/acceleration_distance_count;




%%%%%%第一个卸粮点

%%%第一个卸粮点坐标
unload_x1 = 10;
unload_y1 = 14;
direction_unload1 = atan((unload_y1 - ay)/(unload_x1 - ax));
%%%第一个返回点坐标
unloadback_x1 = 10;
unloadback_y1 = 25;
direction_unloadback1 = atan((unloadback_y1 - ay)/(unloadback_x1 - ax));
%%%从停车点到第一个卸粮点的直线距离
straight_unload1_distance = sqrt((unload_x1 - ax)^2 + (unload_y1 - ay)^2);
%%%从第一个返回点到停车点的直线距离
straight_unloadback1_distance = sqrt((unloadback_x1 - ax)^2 + (unloadback_y1 - ay)^2);
%%%第一个卸粮点后的剩余直线路径距离
workingafter_unload1_distance = distance - unload_x1;
%%%第一个返回点前的剩余直线距离
workingbefore_unloadback1_distance = distance - unloadback_x1;


%%%把距离转换成工作点数
unload1point = floor(straight_unload1_distance/((workingvelocity*1000/3600)/(1/T)));
unloadback1point = floor(straight_unloadback1_distance/((workingvelocity*1000/3600)/(1/T)));

workingpathpoint_afterunload1 = floor(workingafter_unload1_distance/((workingvelocity*1000/3600)/(1/T)));
workingpathpoint_beforeunloadback1 = floor(workingbefore_unloadback1_distance/((workingvelocity*1000/3600)/(1/T)));

%%%%%%第一个卸粮点设置结束



%%%%%%第二个卸粮点

%%%第二个卸粮点坐标
unload_x2 = 90;
unload_y2 = 47;
direction_unload2 = atan((unload_y2 - ay)/(unload_x2 - ax));
%%%第二个返回点坐标
unloadback_x2 = 250;
unloadback_y2 = 58;
direction_unloadback2 = atan((unloadback_y2 - ay)/(unloadback_x2 - ax));
%%%从停车点到第二个卸粮点的直线距离
straight_unload2_distance = sqrt((unload_x2 - ax)^2 + (unload_y2 - ay)^2);
%%%从第二个返回点到停车点的直线距离
straight_unloadback2_distance = sqrt((unloadback_x2 - ax)^2 + (unloadback_y2 - ay)^2);
%%%第二个卸粮点后的剩余直线路径距离
workingafter_unload2_distance = distance - unload_x2;
%%%第二个返回点前的剩余直线距离
workingbefore_unloadback2_distance = distance - unloadback_x2;


%%%把距离转换成工作点数
unload2point = floor(straight_unload2_distance/((workingvelocity*1000/3600)/(1/T)));
unloadback2point = floor(straight_unloadback2_distance/((workingvelocity*1000/3600)/(1/T))) - 50;

workingpathpoint_afterunload2 = floor(workingafter_unload2_distance/((workingvelocity*1000/3600)/(1/T)));
workingpathpoint_beforeunloadback2 = floor(workingbefore_unloadback2_distance/((workingvelocity*1000/3600)/(1/T)));

%%%%%%第二个卸粮点设置结束


%%%%%设置卸粮点数组
unloadpoint = [unload1point,unload2point];
direction_unload = [direction_unload1,direction_unload2];
workingpathpoint_afterunload = [workingpathpoint_afterunload1,workingpathpoint_afterunload2];
workingafter_unload_distance = [workingafter_unload1_distance,workingafter_unload2_distance];
workingpathpoint_beforeunloadback = [workingpathpoint_beforeunloadback1,workingpathpoint_beforeunloadback2];
direction_unloadback = [direction_unloadback1,direction_unloadback2];
unloadbackpoint = [unloadback1point,unloadback2point];





%%%生成路径
global xstart ystart fistart xo yo ;
xstart=ax;
ystart=ay;
fistart=direction;
point=0;
turningflag=initialturning;

for i=1:1:2
    xstart=ax;
    ystart=ay;
    for j = 1:1:unloadpoint(i)
        if j == 1
            r(point+j,1) = point + j;
            r(point+j,2) = xstart;  %x坐标
            r(point+j,3) = ystart;  %y坐标
            r(point+j,4) = direction_unload(i); %一开始的方向
            r(point+j,5) = 0;       %期望转角
            r(point+j,6) = workingvelocity*1000/3600;   %期望速度
            r(point+j,7) = 3;%直行/转弯/出发/返回标志     直行0，转弯1，转弯直线2，出发3，返回4
            r(point+j,8) = (workingvelocity*1000/3600)/(1/T);%前往卸粮点直线段两轨迹点间的距离
        else
            r(point+j,1) = point + j;
            r(point+j,2) = r(point+j-1,2)+(workingvelocity*1000/3600)/(1/T)*cos(direction_unload(i));%x坐标
            r(point+j,3) = r(point+j-1,3)+(workingvelocity*1000/3600)/(1/T)*sin(direction_unload(i));%y坐标
            r(point+j,4) = direction_unload(i); %一开始的方向(沿着直线一直前进)
            r(point+j,5) = 0;       %期望转角
            if j<=acceleration_distance_count
                r(point+j,6)=r(point+j-1,6)+add_velocity;%期望速度
            elseif j>workingpathpoint-acceleration_distance_count
                r(point+j,6)=r(point+j-1,6)-add_velocity;%期望速度
            else
                r(point+j,6)=workingvelocity*1000/3600;
            end
            r(point+j,7) = 3;
            r(point+j,8) = (workingvelocity*1000/3600)/(1/T);%前往卸粮点直线段两轨迹点间的距离
        end
    end
    point = point + unloadpoint(i);

    fistart = 0;

    xstart = r(point,2);
    ystart = r(point,3);


    for j=1:1:workingpathpoint_afterunload(1)
        if j==1
            if i>1
                %                 xstart=xo+R*cos(fistart);
                %                 ystart=yo+R*sin(fistart);
                xstart = r(point,2);
                ystart = r(point,3);

%                 if turningflag==0%%（0为逆时针方向，1为顺时针方向）
%                     fistart=fistart+pi/2;
%                 end
%                 if turningflag==1%%（0为逆时针方向，1为顺时针方向）
%                     fistart=fistart-pi/2;
%                 end
%                 if turningflag==0
%                     turningflag=1;
%                 elseif turningflag==1
%                     turningflag=0;
%                 end
            end
            r(point+j,1)=point+j;
            r(point+j,2)=xstart;%x坐标
            r(point+j,3)=ystart;%y坐标
            r(point+j,4)=fistart;
            r(point+j,5)=0;%期望转角
            r(point+j,6)=turningvelocity*1000/3600;%期望速度
            r(point+j,7)=0;%直行/转弯标志
            r(point+j,8)=(workingvelocity*1000/3600)/(1/T);%工作直线段两轨迹点间的距离
        else
            r(point+j,1)=point+j;
            r(point+j,2)=r(point+j-1,2)+(workingvelocity*1000/3600)/(1/T)*cos(fistart);%x坐标
            r(point+j,3)=r(point+j-1,3)+(workingvelocity*1000/3600)/(1/T)*sin(fistart);%y坐标
            r(point+j,4)=fistart;
            r(point+j,5)=0;%期望转角
            if j<=acceleration_distance_count
                r(point+j,6)=r(point+j-1,6)+add_velocity;%期望速度
            elseif j>workingpathpoint_afterunload(i)-acceleration_distance_count
                r(point+j,6)=r(point+j-1,6)-add_velocity;%期望速度
            else
                r(point+j,6)=workingvelocity*1000/3600;
            end
            r(point+j,7)=0;
            r(point+j,8)=(workingvelocity*1000/3600)/(1/T);%工作直线段两轨迹点间的距离
        end
    end
    point=point+workingpathpoint_afterunload(i);
    %         if i==row
    %             break;
    %         end



    for j=1:1:quadrant
        if j==1
            xstart=xstart+workingafter_unload_distance(i)*cos(fistart);
            ystart=ystart+workingafter_unload_distance(i)*sin(fistart);
            r(point+j,1)=point+j;
            r(point+j,2)=xstart;%x坐标
            r(point+j,3)=ystart;%y坐标
            r(point+j,4)=fistart;
            %                 r(point+j,5)=-atan(L/R);%期望转角
            r(point+j,6)=turningvelocity*1000/3600;%期望速度
            r(point+j,7)=1;
            r(point+j,8)=(turningvelocity*1000/3600)/(1/T);%转弯处两轨迹点间的弧长
            if turningflag==0%%（0为逆时针方向，1为顺时针方向）
                r(point+j,5)=atan(L/R);%期望转角
                xo=xstart-R*sin(fistart);%%转向圆心的x坐标
                yo=ystart+R*cos(fistart);%%转向圆心的y坐标
            end
            if turningflag==1%%（0为逆时针方向，1为顺时针方向）
                r(point+j,5)=-atan(L/R);%期望转角
                xo=xstart+R*sin(fistart);%%转向圆心的x坐标
                yo=ystart-R*cos(fistart);%%转向圆心的y坐标
            end
        end
        if turningflag==0%%（0为逆时针方向，1为顺时针方向）
            r(point+j,1)=point+j;
            r(point+j,2)=xo+R*sin(r(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
            r(point+j,3)=yo-R*cos(r(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
            r(point+j,4)=r(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T)));
            r(point+j,5)=atan(L/R);%期望转角
            r(point+j,6)=turningvelocity*1000/3600;%期望速度
            r(point+j,7)=1;
            r(point+j,8)=(turningvelocity*1000/3600)/(1/T);%转弯处两轨迹点间的弧长
        end
        if turningflag==1%%（0为逆时针方向，1为顺时针方向）
            r(point+j,1)=point+j;
            r(point+j,2)=xo-R*sin(r(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
            r(point+j,3)=yo+R*cos(r(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
            r(point+j,4)=r(point+j-1,4)-2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T)));
            r(point+j,5)=-atan(L/R);%期望转角
            r(point+j,6)=turningvelocity*1000/3600;%期望速度
            r(point+j,7)=1;
            r(point+j,8)=(turningvelocity*1000/3600)/(1/T);%转弯处两轨迹点间的弧长
        end
    end
    %         plot(r(:,2),r(:,3))
    point=point+quadrant;

    for j=1:1:turningstraightlinepoint
        if j==1
            xstart=xo+R*cos(fistart);
            ystart=yo+R*sin(fistart);
            if turningflag==0%%（0为逆时针方向，1为顺时针方向）
                fistart=fistart+pi/2;
            end
            if turningflag==1%%（0为逆时针方向，1为顺时针方向）
                fistart=fistart-pi/2;
            end
            r(point+j,1)=point+j;
            r(point+j,2)=xstart;
            r(point+j,3)=ystart;
            r(point+j,4)=fistart;
            r(point+j,5)=0;%期望转角
            r(point+j,6)=turningvelocity*1000/3600;%期望速度
            r(point+j,7)=2;
            r(point+j,8)=(turningvelocity*1000/3600)/(1/T);%转弯直线处两轨迹点间的距离
        else
            r(point+j,1)=point+j;
            r(point+j,2)=r(point+j-1,2)+(turningvelocity*1000/3600)/(1/T)*cos(fistart);%x坐标
            r(point+j,3)=r(point+j-1,3)+(turningvelocity*1000/3600)/(1/T)*sin(fistart);%y坐标
            r(point+j,4)=fistart;
            r(point+j,5)=0;%期望转角
            r(point+j,6)=turningvelocity*1000/3600;%期望速度
            r(point+j,7)=2;
            r(point+j,8)=(turningvelocity*1000/3600)/(1/T);%转弯直线处两轨迹点间的距离
        end
    end
    %          plot(r(:,2),r(:,3))
    point=point+turningstraightlinepoint;

    for j=1:1:quadrant
        if j==1
            xstart=xstart+(workingdistance-2*R)*cos(fistart);
            ystart=ystart+(workingdistance-2*R)*sin(fistart);
            r(point+j,1)=point+j;
            r(point+j,2)=xstart;
            r(point+j,3)=ystart;
            r(point+j,4)=fistart;
            %                 r(point+j,5)=-atan(L/R);%期望转角
            r(point+j,6)=turningvelocity*1000/3600;%期望速度
            r(point+j,7)=1;
            r(point+j,8)=(turningvelocity*1000/3600)/(1/T);%转弯处两轨迹点间的弧长
            if turningflag==0%%（0为逆时针方向，1为顺时针方向）
                r(point+j,5)=atan(L/R);%期望转角
                xo=xstart-R*sin(fistart);%%转向圆心的x坐标
                yo=ystart+R*cos(fistart);%%转向圆心的y坐标
            end
            if turningflag==1%%（0为逆时针方向，1为顺时针方向）
                r(point+j,5)=-atan(L/R);%期望转角
                xo=xstart+R*sin(fistart);%%转向圆心的x坐标
                yo=ystart-R*cos(fistart);%%转向圆心的y坐标
            end
        end
        if turningflag==0%%（0为逆时针方向，1为顺时针方向）
            r(point+j,1)=point+j;
            r(point+j,2)=xo+R*sin(r(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
            r(point+j,3)=yo-R*cos(r(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
            r(point+j,4)=r(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T)));
            r(point+j,5)=atan(L/R);%期望转角
            r(point+j,6)=turningvelocity*1000/3600;%期望速度
            r(point+j,7)=1;
            r(point+j,8)=(turningvelocity*1000/3600)/(1/T);%转弯处两轨迹点间的弧长
        end
        if turningflag==1%%（0为逆时针方向，1为顺时针方向）
            r(point+j,1)=point+j;
            r(point+j,2)=xo-R*sin(r(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
            r(point+j,3)=yo+R*cos(r(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
            r(point+j,4)=r(point+j-1,4)-2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T)));
            r(point+j,5)=-atan(L/R);%期望转角
            r(point+j,6)=turningvelocity*1000/3600;%期望速度
            r(point+j,7)=1;
            r(point+j,8)=(turningvelocity*1000/3600)/(1/T);%转弯处两轨迹点间的弧长
        end

    end
    point=point+quadrant;

    fistart = pi;

    xstart = r(point,2);
    ystart = r(point,3);


    for j=1:1:workingpathpoint_beforeunloadback(i)
        if j==1
            %             if i>1
            %                 xstart=xo+R*cos(fistart);
            %                 ystart=yo+R*sin(fistart);
            %                 if turningflag==0%%（0为逆时针方向，1为顺时针方向）
            %                     fistart=fistart+pi/2;
            %                 end
            %                 if turningflag==1%%（0为逆时针方向，1为顺时针方向）
            %                     fistart=fistart-pi/2;
            %                 end
            %                 if turningflag==0
            %                     turningflag=1;
            %                 elseif turningflag==1
            %                     turningflag=0;
            %                 end
            %             end
            r(point+j,1)=point+j;
            r(point+j,2)=xstart;%x坐标
            r(point+j,3)=ystart;%y坐标
            r(point+j,4)=fistart;
            r(point+j,5)=0;%期望转角
            r(point+j,6)=turningvelocity*1000/3600;%期望速度
            r(point+j,7)=0;%直行/转弯标志
            r(point+j,8)=(workingvelocity*1000/3600)/(1/T);%工作直线段两轨迹点间的距离
        else
            r(point+j,1)=point+j;
            r(point+j,2)=r(point+j-1,2)+(workingvelocity*1000/3600)/(1/T)*cos(fistart);%x坐标
            r(point+j,3)=r(point+j-1,3)+(workingvelocity*1000/3600)/(1/T)*sin(fistart);%y坐标
            r(point+j,4)=fistart;
            r(point+j,5)=0;%期望转角
            if j<=acceleration_distance_count
                r(point+j,6)=r(point+j-1,6)+add_velocity;%期望速度
            elseif j>workingpathpoint_beforeunloadback(i)-acceleration_distance_count
                r(point+j,6)=r(point+j-1,6)-add_velocity;%期望速度
            else
                r(point+j,6)=workingvelocity*1000/3600;
            end
            r(point+j,7)=0;
            r(point+j,8)=(workingvelocity*1000/3600)/(1/T);%工作直线段两轨迹点间的距离
        end
    end
    point=point+workingpathpoint_beforeunloadback(i);



    fistart = pi + direction_unloadback(i);

    xstart = r(point-1,2);
    ystart = r(point-1,3);

    for j=1:1:unloadbackpoint(i)
        if j==1
            %                         if i>1
            %                             xstart=xo+R*cos(fistart);
            %                             ystart=yo+R*sin(fistart);
            %                             if turningflag==0%%（0为逆时针方向，1为顺时针方向）
            %                                 fistart=fistart+pi/2;
            %                             end
            %                             if turningflag==1%%（0为逆时针方向，1为顺时针方向）
            %                                 fistart=fistart-pi/2;
            %                             end
            %                             if turningflag==0
            %                                 turningflag=1;
            %                             elseif turningflag==1
            %                                 turningflag=0;
            %                             end
            %                         end
            r(point+j,1)=point+j;
            r(point+j,2)=xstart;%x坐标
            r(point+j,3)=ystart;%y坐标
            r(point+j,4)=fistart;
            r(point+j,5)=0;%期望转角
            r(point+j,6)=workingvelocity*1000/3600;%期望速度
            r(point+j,7)=4;%直行/转弯标志
            r(point+j,8)=(workingvelocity*1000/3600)/(1/T);%工作直线段两轨迹点间的距离
        else
            r(point+j,1)=point+j;
            r(point+j,2)=r(point+j-1,2)+(workingvelocity*1000/3600)/(1/T)*cos(fistart);%x坐标
            r(point+j,3)=r(point+j-1,3)+(workingvelocity*1000/3600)/(1/T)*sin(fistart);%y坐标
            r(point+j,4)=fistart;
            r(point+j,5)=0;%期望转角
            if j<=acceleration_distance_count
                r(point+j,6)=r(point+j-1,6)+add_velocity;%期望速度
            elseif j>unloadbackpoint(i)-acceleration_distance_count
                r(point+j,6)=r(point+j-1,6)-add_velocity;%期望速度
            else
                r(point+j,6)=workingvelocity*1000/3600;
            end
            r(point+j,7)=4;
            r(point+j,8)=(workingvelocity*1000/3600)/(1/T);%工作直线段两轨迹点间的距离
        end
    end
    point=point+unloadbackpoint(i);

end



figure(3);
axis([-10,400,-50,200]);
hold on;
plot(r(:,2),r(:,3));



global U;%%%控制器控制量（非差分量）及其初始化
U=[0;0];
str = [];  % 设置一个str空矩阵
ts = [0.05 0];  % sample time : [period, offset]
% 初始化子函数结束









% 更新离散量状态量子函数
function sys = mdlUpdates(t, x, u)
sys = x;
% 离散量状态量子函数结束











%计算输出子函数
function sys = mdlOutputs(t, x, u)

global Np Nc Nx Nu;
global a b u_piao;
global kesi;
global T;
global count skipflag1 time_record L T1 data_record data_record2;

count=count+1;
if count==1
    skipflag1=1;
else
    skipflag1=0;
end
global u_piao;
global r;
global U t_d targetp targetx targety targett targetv;%%分别对应代价函数中变量UU、tt_dd、uuu、targetpp、targetxx、targetyy
%     U(1)=u(4);
%     U(2)=u(5)*pi/180;


t_d=u(3);
targetp=zeros(1,Np);%%定义期望轨迹点（方向）
targetx=zeros(1,Np);%%定义期望轨迹点（x坐标）
targety=zeros(1,Np);%%定义期望轨迹点（y坐标）
targett=zeros(1,Np);%%定义期望轨迹点（转角）
targetv=zeros(1,Np);%%定义期望轨迹点（速度）




global point2;
global point21;     %%主机的point2


% %%%%%%往后推10m的代码
%     ten_meters_distance_count=0;
%     ten_meters_distance=0;
%     for i=point21:-1:1
%         ten_meters_distance=ten_meters_distance+r(i,8);
%         if ten_meters_distance>=10
%             ten_meters_distance_count=i;
%             break;
%         elseif ten_meters_distance<10&&i==1
%             ten_meters_distance_count=1;
% 
%         end
% 
%     end
%     point2=ten_meters_distance_count;



%%%%%%%%改这里！！

% 
% point2 = point2 +1;


    if point2 < 100
        point2 = point2 + 1;
    else 
            if point2-(5/T1)<1
                 for i=1:1:point2+(5/T1)
                    if sqrt((r(i,2)-u(1,1))^2+(r(i,3)-u(2,1))^2)<sqrt((r(point2,2)-u(1,1))^2+(r(point2,3)-u(2,1))^2)
                        point2=i;
                    end
                 end
            else
                  for i=point2-(5/T1):1:point2+(5/T1)
                    if sqrt((r(i,2)-u(1,1))^2+(r(i,3)-u(2,1))^2)<sqrt((r(point2,2)-u(1,1))^2+(r(point2,3)-u(2,1))^2)
                        point2=i;
                    end
                  end
            end  
    end



%     if point2==1
%         for i=1:1:size(r,1)
%             if sqrt((r(i,2)-u(1,1))^2+(r(i,3)-u(2,1))^2)<sqrt((r(point2,2)-u(1,1))^2+(r(point2,3)-u(2,1))^2)
%                 point2=i;
%             end
%         end
%     else 
%             if point2-(5/T1)<1
%                  for i=1:1:point2+(5/T1)
%                     if sqrt((r(i,2)-u(1,1))^2+(r(i,3)-u(2,1))^2)<sqrt((r(point2,2)-u(1,1))^2+(r(point2,3)-u(2,1))^2)
%                         point2=i;
%                     end
%                  end
%             else
%                   for i=point2-(5/T1):1:point2+(5/T1)
%                     if sqrt((r(i,2)-u(1,1))^2+(r(i,3)-u(2,1))^2)<sqrt((r(point2,2)-u(1,1))^2+(r(point2,3)-u(2,1))^2)
%                         point2=i;
%                     end
%                   end
%             end  
%     end





% % 配合转弯的算法，取消注释后即可用，用这个可以让从机在直线跟上主机的索引点
% if r(point2,7) == 0
%     %     if point2==1
%     %         for i=1:1:size(r,1)
%     %             if sqrt((r(i,2)-u(1,1))^2+(r(i,3)-u(2,1))^2)<sqrt((r(point2,2)-u(1,1))^2+(r(point2,3)-u(2,1))^2)
%     %                 point2=i;
%     %             end
%     %         end
%     %     else
%     %             if point2-(5/T1)<1
%     %                  for i=1:1:point2+(5/T1)
%     %                     if sqrt((r(i,2)-u(1,1))^2+(r(i,3)-u(2,1))^2)<sqrt((r(point2,2)-u(1,1))^2+(r(point2,3)-u(2,1))^2)
%     %                         point2=i;
%     %                     end
%     %                  end
%     %             else
%     %                   for i=point2-(5/T1):1:point2+(5/T1)
%     %                     if sqrt((r(i,2)-u(1,1))^2+(r(i,3)-u(2,1))^2)<sqrt((r(point2,2)-u(1,1))^2+(r(point2,3)-u(2,1))^2)
%     %                         point2=i;
%     %                     end
%     %                   end
%     %             end
%     %     end
%     point2=point21;
% else
%     if point2==1
%         for i=1:1:size(r,1)
%             if sqrt((r(i,2)-u(1,1))^2+(r(i,3)-u(2,1))^2)<sqrt((r(point2,2)-u(1,1))^2+(r(point2,3)-u(2,1))^2)
%                 point2=i;
%             end
%         end
%     else
%         if point2-(5/T1)<1
%             for i=1:1:point2+(5/T1)
%                 if sqrt((r(i,2)-u(1,1))^2+(r(i,3)-u(2,1))^2)<sqrt((r(point2,2)-u(1,1))^2+(r(point2,3)-u(2,1))^2)
%                     point2=i;
%                 end
%             end
%         else
%             for i=point2-(5/T1):1:point2+(5/T1)
%                 if sqrt((r(i,2)-u(1,1))^2+(r(i,3)-u(2,1))^2)<sqrt((r(point2,2)-u(1,1))^2+(r(point2,3)-u(2,1))^2)
%                     point2=i;
%                 end
%             end
%         end
%     end
% end








for i=1:1:Np
    targetp(1,i)=r(point2+i*(T1/0.01)-9,4);
    targetx(1,i)=r(point2+i*(T1/0.01)-9,2);
    targety(1,i)=r(point2+i*(T1/0.01)-9,3);
    targett(1,i)=r(point2+i*(T1/0.01)-9,5);
    targetv(1,i)=r(point2+i*(T1/0.01)-9,6);
end


Row = 10;  % 松弛因子
fprintf('Update start, t = %6.3f\n', t)



% 参数设计
kesi=zeros(Nx+Nu,1);
kesi(1)=u(1)-targetx(1,1);%u(1)==X(1);
kesi(2)=u(2)-targety(1,1);%u(2)==X(2)
kesi(3)=t_d-targetp(1,1);%u(3)==X(3)
kesi(4)=U(1);
kesi(5)=U(2);
fprintf('Update start,u(1)=%4.2f\n',U(1))
fprintf('Update start,u(2)=%4.2f\n',U(2))
%矩阵初始化
u_piao=zeros(Nx,Nu);%%%%%%后边用来装优化计算结果（控制增量）
Q=1000*eye(Nx*Np,Nx*Np);
R=50*eye(Nu*Nc);
a=cell(1,Np);
b=cell(1,Np);
for i=1:1:Np
    a{1,i}=[1 0 -targetv(1,i)*sin(targetp(1,i))*T1;
        0 1 targetv(1,i)*cos(targetp(1,i))*T1;
        0 0 1;];%[Nx Nx]
    b{1,i}= [cos(targetp(1,i))*T1 0;
        sin(targetp(1,i))*T1 0;
        tan(targett(1,i))*T1/L targetv(1,i)*T1/(cos(targett(1,i))^2);];%[Nx Nu]
end
% 对应（4.6）中的参数
A=cell(1,Np);
B=cell(1,Np);
for i=1:1:Np
    A_cell=cell(2,2);
    B_cell=cell(2,1);
    A_cell{1,1}=cell2mat(a(1,i));
    A_cell{1,2}=cell2mat(b(1,i));
    A_cell{2,1}=zeros(Nu,Nx);
    A_cell{2,2}=eye(Nu);
    B_cell{1,1}=cell2mat(b(1,i));
    B_cell{2,1}=eye(Nu);
    A{1,i}=cell2mat(A_cell);%[Nx+Nu Nx+Nu]
    B{1,i}=cell2mat(B_cell);%[Nx+Nu Nu]
end
C=[1 0 0 0 0;0 1 0 0 0;0 0 1 0 0;];
PHI_cell=cell(Np,1);
THETA_cell=cell(Np,Nc);
for j=1:1:Np
    if j==1
        PHI_cell{j,1}=C*A{1,j};
    else
        AA=A{1,j};
        for i=j-1:-1:1
            AA=AA*A{1,i};
        end
        PHI_cell{j,1}=C*AA;
    end
    for k=1:1:Nc

        if k==j
            THETA_cell{j,k}=C*B{1,j};
        else if k<j
                AA=A{1,j};
                if k+1<j
                    for i=j-1:-1:k+1
                        AA=AA*A{1,i};
                    end
                end
                THETA_cell{j,k}=C*AA*B{1,k};
        else
            THETA_cell{j,k}=zeros(Nx,Nu);
        end
        end

    end
end
PHI=cell2mat(PHI_cell);%size(PHI)=[Nx*Np Nx+Nu]
THETA=cell2mat(THETA_cell);%size(THETA)=[Nx*Np Nu*Nc]
H_cell=cell(2,2);
H_cell{1,1}=THETA'*Q*THETA+R;
H_cell{1,2}=zeros(Nu*Nc,1);
H_cell{2,1}=zeros(1,Nu*Nc);
H_cell{2,2}=Row;
H=cell2mat(H_cell);

error=PHI*kesi;
f_cell=cell(1,2);
f_cell{1,1}=2*error'*Q*THETA;
f_cell{1,2}=0;
%f=cell2mat(f_cell)';
f=cell2mat(f_cell);
% %以下为约束生成区域
%不等式约束
A_t=zeros(Nc,Nc);%见falcone论文P181
for p=1:1:Nc
    for q=1:1:Nc
        if q<=p
            A_t(p,q)=1;
        else
            A_t(p,q)=0;
        end
    end
end
A_l=kron(A_t,eye(Nu));%对应于falcone论文约束处理的矩阵A，求克罗内克积
Ut=kron(ones(Nc,1),U);%此处感觉论文里的克罗内科积有问题，暂时交换顺序
% % % umin=[-0.2;-0.54;];%维数与控制变量的个数相同
% % % umax=[0.2;0.332;];

umin=[-2;-1;];%维数与控制变量的个数相同
umax=[4;1;];
delta_umin=[-0.3;-0.03;];%delta_umin=[0.05;-0.0082;];原代码有错，速度变化下界没加负号
delta_umax=[0.5;0.03];
% umin=[-0.2;-0.54;];%维数与控制变量的个数相同
% umax=[0.2;0.332;];
% delta_umin=[-0.05;-0.0082;];%delta_umin=[0.05;-0.0082;];原代码有错，速度变化下界没加负号
% delta_umax=[0.05;0.0082];

Umin=kron(ones(Nc,1),umin);
Umax=kron(ones(Nc,1),umax);
A_cons_cell={A_l zeros(Nu*Nc,1);-A_l zeros(Nu*Nc,1)};
b_cons_cell={Umax-Ut;-Umin+Ut};
A_cons=cell2mat(A_cons_cell);%(求解方程)状态量不等式约束增益矩阵，转换为绝对值的取值范围
b_cons=cell2mat(b_cons_cell);%(求解方程)状态量不等式约束的取值
%状态量约束
M=10;
delta_Umin=kron(ones(Nc,1),delta_umin);
delta_Umax=kron(ones(Nc,1),delta_umax);
lb=[delta_Umin;0];%(求解方程)状态量下界，包含控制时域内控制增量和松弛因子
ub=[delta_Umax;M];%(求解方程)状态量上界，包含控制时域内控制增量和松弛因子

% %开始求解过程
%options = optimset('Algorithm','active-set');新版quadprog不能用有效集法，这里选内点法
options = optimset('Algorithm','interior-point-convex');
% [X,fval,exitflag]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub,[],options);
[X,fval,exitflag]=quadprog(H,f,A_cons,b_cons,[],[],[],[],[],options);

% %判断是否到达终点
% f=0;
% dis=norm([u(1)-20 u(2)-66]);
% if(abs(dis)<0.2)
%     f=1;
% end


% %计算输出
u_piao(1)=X(1);
u_piao(2)=X(2);
U(1)=kesi(4)+u_piao(1);
U(2)=kesi(5)+u_piao(2);
u_real(1)=U(1)+targetv(1,1);
u_real(2)=U(2)+targett(1,1);

% %判断是否到达终点，停止
% if f==1
%     u_real(1)=0;
%     u_real(2)=0;
% end

%%判断运粮车是否可以出发，若收割机未到达相应位置，则运粮车一直停止

if point21 <= 23500
    u_real(1)=0;
    u_real(2)=0;
end



global quadrant turningstraightlinepoint;
if r(point21,7)~=0&&r(point2,7)~=0&&(abs(point21-point2)<(2*quadrant+turningstraightlinepoint))
    u_real(1)=0;
    u_real(2)=0;
end
% if point2==1
%     u_real(1)=0;
%     u_real(2)=0;
% end
%  global begin_ten_distance_count
% if point21<=begin_ten_distance_count
%     u_real(1)=0;
%     u_real(2)=0;
% end

sys=u_real;
% toc
% 输出结束