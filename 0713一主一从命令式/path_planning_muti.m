clear all;
global T1 L;
T1=0.1;%控制器计算周期
L=2.05;%农机车辆轴距
global Nx Nu Np Nc;
Nx=3;%状态量的个数
Nu=2;%控制量的个数
Np=10;%预测步长
Nc=10;%控制步长
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
unloadback1point = floor(straight_unloadback1_distance/((workingvelocity*1000/3600)/(1/T))) - 50;

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

%%%%%%%静态画图
figure(3);
axis([-50,400,-50,300]);
hold on;
plot(r(:,2),r(:,3));
