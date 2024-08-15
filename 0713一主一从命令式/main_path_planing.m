global T1 L;
T1=0.1;%控制器计算周期
L=2.05;%农机车辆轴距
global Nx1 Nu1 Np1 Nc1;
    Nx1=3;%状态量的个数
    Nu1=2;%控制量的个数
    Np1=10;%预测步长
    Nc1=10;%控制步长
global point21 count1 ;
count1=0;%控制器求解次数记数
% point_flag=0;%期望轨迹初次计算标志
point21=1;%%期望点初始化
% x0=zeros(1,Nc1*2);%%优化初始值初始化

global r1 ;
%     T_all=15;%临时设定，总的仿真时间，主要功能是防止计算期望轨迹越界
%     %半径为25m的圆形轨迹，速度为5m/秒，根据全局期望轨迹生成期望点
%     i=1;
% 
%     for TT=T1:T1:T_all
%         r1(i,1)=i;  
%         r1(i,2)=5*sin(0.4*TT);%x坐标
%         r1(i,3)=5-5*cos(0.4*TT);%y坐标
%         %%%期望方向应该需要一定转换？？？？？？？？？？？？？？？？？？？？
%         r1(i,4)=0.4*TT;%fai坐标
%         i=i+1;
%     end
%%%设置A点和B点坐标
ax=0;
ay=-4;
bx=300;
by=-4;
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
%%%确定AB线长度
distance=sqrt(abx^2+aby^2);
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
            
            
       
       
       
%%%生成路径
global xstart1 ystart1 fistart1 xo1 yo1 ;
xstart1=ax;
ystart1=ay;
fistart1=direction;
point=0;
turningflag=initialturning;

for j=1:1:workingpathpoint
            if j==1
                if i>1
                    xstart1=xo1+R*cos(fistart1);
                    ystart1=yo1+R*sin(fistart1);
                    if turningflag==0%%（0为逆时针方向，1为顺时针方向）
                        fistart1=fistart1+pi/2;
                    end
                    if turningflag==1%%（0为逆时针方向，1为顺时针方向）
                        fistart1=fistart1-pi/2;
                    end
                    if turningflag==0
                        turningflag=1;
                    elseif turningflag==1
                        turningflag=0;
                    end
                end
                r1(point+j,1)=point+j;  
                r1(point+j,2)=xstart1;%x坐标
                r1(point+j,3)=ystart1;%y坐标
                r1(point+j,4)=fistart1;
                r1(point+j,5)=0;%期望转角
                r1(point+j,6)=turningvelocity*1000/3600;%期望速度
                r1(point+j,7)=0;%直行/转弯标志
                r1(point+j,8)=(workingvelocity*1000/3600)/(1/T);%工作直线段两轨迹点间的距离
            else
                r1(point+j,1)=point+j;  
                r1(point+j,2)=r1(point+j-1,2)+(workingvelocity*1000/3600)/(1/T)*cos(fistart1);%x坐标
                r1(point+j,3)=r1(point+j-1,3)+(workingvelocity*1000/3600)/(1/T)*sin(fistart1);%y坐标
                r1(point+j,4)=fistart1;
                r1(point+j,5)=0;%期望转角
                if j<=acceleration_distance_count
                    r1(point+j,6)=r1(point+j-1,6)+add_velocity;%期望速度
                elseif j>workingpathpoint-acceleration_distance_count
                    r1(point+j,6)=r1(point+j-1,6)-add_velocity;%期望速度
                else
                    r1(point+j,6)=workingvelocity*1000/3600;
                end
                r1(point+j,7)=0;
                
            end
        end
        point=point+workingpathpoint;
        for j=1:1:quadrant
            if j==1
                xstart1=xstart1+distance*cos(fistart1);
                ystart1=ystart1+distance*sin(fistart1);
                r1(point+j,1)=point+j;  
                r1(point+j,2)=xstart1;%x坐标
                r1(point+j,3)=ystart1;%y坐标
                r1(point+j,4)=fistart1;
%                 r1(point+j,5)=-atan(L/R);%期望转角
                r1(point+j,6)=turningvelocity*1000/3600;%期望速度
                r1(point+j,7)=1;
               
                if turningflag==0%%（0为逆时针方向，1为顺时针方向）
                    r1(point+j,5)=atan(L/R);%期望转角
                    xo1=xstart1-R*sin(fistart1);%%转向圆心的x坐标
                    yo1=ystart1+R*cos(fistart1);%%转向圆心的y坐标
                end
                if turningflag==1%%（0为逆时针方向，1为顺时针方向）
                    r1(point+j,5)=-atan(L/R);%期望转角
                    xo1=xstart1+R*sin(fistart1);%%转向圆心的x坐标
                    yo1=ystart1-R*cos(fistart1);%%转向圆心的y坐标
                end
            end
                if turningflag==0%%（0为逆时针方向，1为顺时针方向）
                    r1(point+j,1)=point+j;  
                    r1(point+j,2)=xo1+R*sin(r1(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
                    r1(point+j,3)=yo1-R*cos(r1(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
                    r1(point+j,4)=r1(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T)));
                    r1(point+j,5)=atan(L/R);%期望转角
                    r1(point+j,6)=turningvelocity*1000/3600;%期望速度
                    r1(point+j,7)=1;
                end
                if turningflag==1%%（0为逆时针方向，1为顺时针方向）
                    r1(point+j,1)=point+j;  
                    r1(point+j,2)=xo1-R*sin(r1(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
                    r1(point+j,3)=yo1+R*cos(r1(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
                    r1(point+j,4)=r1(point+j-1,4)-2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T)));
                    r1(point+j,5)=-atan(L/R);%期望转角
                    r1(point+j,6)=turningvelocity*1000/3600;%期望速度
                    r1(point+j,7)=1;
                end
        end
%         plot(r1(:,2),r1(:,3))
        point=point+quadrant;
        
        for j=1:1:turningstraightlinepoint
            if j==1
                xstart1=xo1+R*cos(fistart1);
                ystart1=yo1+R*sin(fistart1);
                if turningflag==0%%（0为逆时针方向，1为顺时针方向）
                    fistart1=fistart1+pi/2;
                end
                if turningflag==1%%（0为逆时针方向，1为顺时针方向）
                    fistart1=fistart1-pi/2;
                end
                r1(point+j,1)=point+j;  
                r1(point+j,2)=xstart1;
                r1(point+j,3)=ystart1;
                r1(point+j,4)=fistart1;
                r1(point+j,5)=0;%期望转角
                r1(point+j,6)=turningvelocity*1000/3600;%期望速度
                r1(point+j,7)=2;
            else
                r1(point+j,1)=point+j;  
                r1(point+j,2)=r1(point+j-1,2)+(turningvelocity*1000/3600)/(1/T)*cos(fistart1);%x坐标
                r1(point+j,3)=r1(point+j-1,3)+(turningvelocity*1000/3600)/(1/T)*sin(fistart1);%y坐标
                r1(point+j,4)=fistart1;
                r1(point+j,5)=0;%期望转角
                r1(point+j,6)=turningvelocity*1000/3600;%期望速度
                r1(point+j,7)=2;
            end
        end
%          plot(r1(:,2),r1(:,3))
        point=point+turningstraightlinepoint;
        
        for j=1:1:quadrant
            if j==1
                xstart1=xstart1+(workingdistance-2*R)*cos(fistart1);
                ystart1=ystart1+(workingdistance-2*R)*sin(fistart1);
                r1(point+j,1)=point+j;  
                r1(point+j,2)=xstart1;
                r1(point+j,3)=ystart1;
                r1(point+j,4)=fistart1;
%                 r1(point+j,5)=-atan(L/R);%期望转角
                r1(point+j,6)=turningvelocity*1000/3600;%期望速度
                r1(point+j,7)=1;
                if turningflag==0%%（0为逆时针方向，1为顺时针方向）
                    r1(point+j,5)=atan(L/R);%期望转角
                    xo1=xstart1-R*sin(fistart1);%%转向圆心的x坐标
                    yo1=ystart1+R*cos(fistart1);%%转向圆心的y坐标
                end
                if turningflag==1%%（0为逆时针方向，1为顺时针方向）
                    r1(point+j,5)=-atan(L/R);%期望转角
                    xo1=xstart1+R*sin(fistart1);%%转向圆心的x坐标
                    yo1=ystart1-R*cos(fistart1);%%转向圆心的y坐标
                end
            end
                if turningflag==0%%（0为逆时针方向，1为顺时针方向）
                    r1(point+j,1)=point+j;  
                    r1(point+j,2)=xo1+R*sin(r1(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
                    r1(point+j,3)=yo1-R*cos(r1(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
                    r1(point+j,4)=r1(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T)));
                    r1(point+j,5)=atan(L/R);%期望转角
                    r1(point+j,6)=turningvelocity*1000/3600;%期望速度
                    r1(point+j,7)=1;
                end
                if turningflag==1%%（0为逆时针方向，1为顺时针方向）
                    r1(point+j,1)=point+j;  
                    r1(point+j,2)=xo1-R*sin(r1(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
                    r1(point+j,3)=yo1+R*cos(r1(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
                    r1(point+j,4)=r1(point+j-1,4)-2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T)));
                    r1(point+j,5)=-atan(L/R);%期望转角
                    r1(point+j,6)=turningvelocity*1000/3600;%期望速度
                    r1(point+j,7)=1;
                end
            
        end
        point=point+quadrant;


        figure(3);
             plot(r1(:,2),r1(:,3));
             hold on;
