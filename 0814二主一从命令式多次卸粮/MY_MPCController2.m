%主机2（收割机2）
function[sys, x0, str, ts] = MY_MPCController2(t, x, u, flag)
switch flag
    case 0   %初始化
        [sys, x0, str, ts] = mdlInitializeSizes;  %初始化
    case 2  %更新离散状态
        sys = mdlUpdates(t, x, u);  
    case 3  %计算输出
        sys = mdlOutputs(t, x, u);
    case {1, 5, 9, 10}  % Unused flags
        sys = [];
    otherwise  %未知的flag
        error(['unhandled flag =' ,num2str(flag)]);  %Error handling
end
% s函数主程序结束

% 初始化子函数
function[sys, x0, str, ts, point] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates = 0;  % 连续状态量的个数
sizes.NumDiscStates = 3;  % 离散状态量的个数
sizes.NumOutputs = 3;  % 输出量的个数
sizes.NumInputs = 3;  %输入量的个数
sizes.DirFeedthrough = 1;  % 矩阵D非空， 直接贯通标志
sizes.NumSampleTimes = 1;  % 采样时间的个数
sys = simsizes(sizes);
x0 = [0; 0; 0];  % 状态量初始化
global T1 L;
T1=0.1;%控制器计算周期
L=2.05;%农机车辆轴距
global Nx2 Nu2 Np2 Nc2;
    Nx2=3;%状态量的个数
    Nu2=2;%控制量的个数
    Np2=10;%预测步长
    Nc2=10;%控制步长
global point22 count2 ;
count2=0;%控制器求解次数记数
% point_flag=0;%期望轨迹初次计算标志
point22=1;%%期望点初始化
% x0=zeros(1,Nc1*2);%%优化初始值初始化

global r2 ;
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
ay=-20;
bx=300;
by=-20;
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
global xstart2 ystart2 fistart2 xo2 yo2 ;
xstart2=ax;
ystart2=ay;
fistart2=direction;
point=0;
turningflag=initialturning;
    for i=1:1:row
        for j=1:1:workingpathpoint
            if j==1
                if i>1
                    xstart2=xo2+R*cos(fistart2);
                    ystart2=yo2+R*sin(fistart2);
                    if turningflag==0%%（0为逆时针方向，1为顺时针方向）
                        fistart2=fistart2+pi/2;
                    end
                    if turningflag==1%%（0为逆时针方向，1为顺时针方向）
                        fistart2=fistart2-pi/2;
                    end
                    if turningflag==0
                        turningflag=1;
                    elseif turningflag==1
                        turningflag=0;
                    end
                end
                r2(point+j,1)=point+j;  
                r2(point+j,2)=xstart2;%x坐标
                r2(point+j,3)=ystart2;%y坐标
                r2(point+j,4)=fistart2;
                r2(point+j,5)=0;%期望转角
                r2(point+j,6)=turningvelocity*1000/3600;%期望速度
                r2(point+j,7)=0;%直行/转弯标志
                r2(point+j,8)=(workingvelocity*1000/3600)/(1/T);%工作直线段两轨迹点间的距离
            else
                r2(point+j,1)=point+j;  
                r2(point+j,2)=r2(point+j-1,2)+(workingvelocity*1000/3600)/(1/T)*cos(fistart2);%x坐标
                r2(point+j,3)=r2(point+j-1,3)+(workingvelocity*1000/3600)/(1/T)*sin(fistart2);%y坐标
                r2(point+j,4)=fistart2;
                r2(point+j,5)=0;%期望转角
                if j<=acceleration_distance_count
                    r2(point+j,6)=r2(point+j-1,6)+add_velocity;%期望速度
                elseif j>workingpathpoint-acceleration_distance_count
                    r2(point+j,6)=r2(point+j-1,6)-add_velocity;%期望速度
                else
                    r2(point+j,6)=workingvelocity*1000/3600;
                end
                r2(point+j,7)=0;
                
            end
        end
        point=point+workingpathpoint;
        if i==row
            break;
        end
        for j=1:1:quadrant
            if j==1
                xstart2=xstart2+distance*cos(fistart2);
                ystart2=ystart2+distance*sin(fistart2);
                r2(point+j,1)=point+j;  
                r2(point+j,2)=xstart2;%x坐标
                r2(point+j,3)=ystart2;%y坐标
                r2(point+j,4)=fistart2;
%                 r1(point+j,5)=-atan(L/R);%期望转角
                r2(point+j,6)=turningvelocity*1000/3600;%期望速度
                r2(point+j,7)=1;
               
                if turningflag==0%%（0为逆时针方向，1为顺时针方向）
                    r2(point+j,5)=atan(L/R);%期望转角
                    xo2=xstart2-R*sin(fistart2);%%转向圆心的x坐标
                    yo2=ystart2+R*cos(fistart2);%%转向圆心的y坐标
                end
                if turningflag==1%%（0为逆时针方向，1为顺时针方向）
                    r2(point+j,5)=-atan(L/R);%期望转角
                    xo2=xstart2+R*sin(fistart2);%%转向圆心的x坐标
                    yo2=ystart2-R*cos(fistart2);%%转向圆心的y坐标
                end
            end
                if turningflag==0%%（0为逆时针方向，1为顺时针方向）
                    r2(point+j,1)=point+j;  
                    r2(point+j,2)=xo2+R*sin(r2(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
                    r2(point+j,3)=yo2-R*cos(r2(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
                    r2(point+j,4)=r2(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T)));
                    r2(point+j,5)=atan(L/R);%期望转角
                    r2(point+j,6)=turningvelocity*1000/3600;%期望速度
                    r2(point+j,7)=1;
                end
                if turningflag==1%%（0为逆时针方向，1为顺时针方向）
                    r2(point+j,1)=point+j;  
                    r2(point+j,2)=xo2-R*sin(r2(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
                    r2(point+j,3)=yo2+R*cos(r2(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
                    r2(point+j,4)=r2(point+j-1,4)-2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T)));
                    r2(point+j,5)=-atan(L/R);%期望转角
                    r2(point+j,6)=turningvelocity*1000/3600;%期望速度
                    r2(point+j,7)=1;
                end
        end
%         plot(r1(:,2),r1(:,3))
        point=point+quadrant;
        
        for j=1:1:turningstraightlinepoint
            if j==1
                xstart2=xo2+R*cos(fistart2);
                ystart2=yo2+R*sin(fistart2);
                if turningflag==0%%（0为逆时针方向，1为顺时针方向）
                    fistart2=fistart2+pi/2;
                end
                if turningflag==1%%（0为逆时针方向，1为顺时针方向）
                    fistart2=fistart2-pi/2;
                end
                r2(point+j,1)=point+j;  
                r2(point+j,2)=xstart2;
                r2(point+j,3)=ystart2;
                r2(point+j,4)=fistart2;
                r2(point+j,5)=0;%期望转角
                r2(point+j,6)=turningvelocity*1000/3600;%期望速度
                r2(point+j,7)=2;
            else
                r2(point+j,1)=point+j;  
                r2(point+j,2)=r2(point+j-1,2)+(turningvelocity*1000/3600)/(1/T)*cos(fistart2);%x坐标
                r2(point+j,3)=r2(point+j-1,3)+(turningvelocity*1000/3600)/(1/T)*sin(fistart2);%y坐标
                r2(point+j,4)=fistart2;
                r2(point+j,5)=0;%期望转角
                r2(point+j,6)=turningvelocity*1000/3600;%期望速度
                r2(point+j,7)=2;
            end
        end
%          plot(r1(:,2),r1(:,3))
        point=point+turningstraightlinepoint;
        
        for j=1:1:quadrant
            if j==1
                xstart2=xstart2+(workingdistance-2*R)*cos(fistart2);
                ystart2=ystart2+(workingdistance-2*R)*sin(fistart2);
                r2(point+j,1)=point+j;  
                r2(point+j,2)=xstart2;
                r2(point+j,3)=ystart2;
                r2(point+j,4)=fistart2;
%                 r1(point+j,5)=-atan(L/R);%期望转角
                r2(point+j,6)=turningvelocity*1000/3600;%期望速度
                r2(point+j,7)=1;
                if turningflag==0%%（0为逆时针方向，1为顺时针方向）
                    r2(point+j,5)=atan(L/R);%期望转角
                    xo2=xstart2-R*sin(fistart2);%%转向圆心的x坐标
                    yo2=ystart2+R*cos(fistart2);%%转向圆心的y坐标
                end
                if turningflag==1%%（0为逆时针方向，1为顺时针方向）
                    r2(point+j,5)=-atan(L/R);%期望转角
                    xo2=xstart2+R*sin(fistart2);%%转向圆心的x坐标
                    yo2=ystart2-R*cos(fistart2);%%转向圆心的y坐标
                end
            end
                if turningflag==0%%（0为逆时针方向，1为顺时针方向）
                    r2(point+j,1)=point+j;  
                    r2(point+j,2)=xo2+R*sin(r2(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
                    r2(point+j,3)=yo2-R*cos(r2(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
                    r2(point+j,4)=r2(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T)));
                    r2(point+j,5)=atan(L/R);%期望转角
                    r2(point+j,6)=turningvelocity*1000/3600;%期望速度
                    r2(point+j,7)=1;
                end
                if turningflag==1%%（0为逆时针方向，1为顺时针方向）
                    r2(point+j,1)=point+j;  
                    r2(point+j,2)=xo2-R*sin(r2(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
                    r2(point+j,3)=yo2+R*cos(r2(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
                    r2(point+j,4)=r2(point+j-1,4)-2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T)));
                    r2(point+j,5)=-atan(L/R);%期望转角
                    r2(point+j,6)=turningvelocity*1000/3600;%期望速度
                    r2(point+j,7)=1;
                end
            
        end
        point=point+quadrant;
%         if turningflag==0%%（0为逆时针方向，1为顺时针方向）
%             fistart1=fistart1+pi/2;
%         end
%         if turningflag==1%%（0为逆时针方向，1为顺时针方向）
%             fistart1=fistart1-pi/2;
%         end
%         if turningflag==0
%             turningflag=1;
%         end
%         if turningflag==1
%             turningflag=0;
%         end
%             plot(r1(:,2),r1(:,3));
    end
             figure(3);
             plot(r2(:,2),r2(:,3));
             hold on;
%             r1(:,3)=r1(:,3)-4;

global U2;%%%控制器控制量（非差分量）及其初始化
U2=[0;0];
str = [];  % 设置一个str空矩阵
ts = [0.05 0];  % sample time : [period, offset]
% 初始化子函数结束

% 更新离散量状态量子函数
function sys = mdlUpdates(t, x, u)
sys = x;
% 离散量状态量子函数结束

%计算输出子函数
function sys = mdlOutputs(t, x, u)

    global Np2 Nc2 Nx2 Nu2;
    global a2 b2 u_piao2;
    global kesi2;
    global T;
    global count2 skipflag1 time_record2 L T1 data_record2 data_record22;
    
    count2=count2+1;
    if count2==1
        skipflag1=1;
    else
        skipflag1=0;
    end
    global u_piao2;
    global r2;
    global U2 t_d targetp targetx targety targett targetv;%%分别对应代价函数中变量UU、tt_dd、uuu、targetpp、targetxx、targetyy
%     U1(1)=u(4);
%     U1(2)=u(5)*pi/180;
    t_d=u(3);
    targetp=zeros(1,Np2);%%定义期望轨迹点（方向）
    targetx=zeros(1,Np2);%%定义期望轨迹点（x坐标）
    targety=zeros(1,Np2);%%定义期望轨迹点（y坐标）
    targett=zeros(1,Np2);%%定义期望轨迹点（转角）
    targetv=zeros(1,Np2);%%定义期望轨迹点（速度）
    global point22;
    if point22==1
        for i=1:1:size(r2,1)
            if sqrt((r2(i,2)-u(1,1))^2+(r2(i,3)-u(2,1))^2)<sqrt((r2(point22,2)-u(1,1))^2+(r2(point22,3)-u(2,1))^2)
                point22=i;
            end
        end
    else 
            if point22-(5/T1)<1
                 for i=1:1:point22+(5/T1)
                    if sqrt((r2(i,2)-u(1,1))^2+(r2(i,3)-u(2,1))^2)<sqrt((r2(point22,2)-u(1,1))^2+(r2(point22,3)-u(2,1))^2)
                        point22=i;
                    end
                 end
            else
                  for i=point22-(5/T1):1:point22+(5/T1)
                    if sqrt((r2(i,2)-u(1,1))^2+(r2(i,3)-u(2,1))^2)<sqrt((r2(point22,2)-u(1,1))^2+(r2(point22,3)-u(2,1))^2)
                        point22=i;
                    end
                  end
            end  
    end
    

for i=1:1:Np2
        targetp(1,i)=r2(point22+i*(T1/0.01)-9,4);
        targetx(1,i)=r2(point22+i*(T1/0.01)-9,2);
        targety(1,i)=r2(point22+i*(T1/0.01)-9,3);
        targett(1,i)=r2(point22+i*(T1/0.01)-9,5);
        targetv(1,i)=r2(point22+i*(T1/0.01)-9,6);
end


Row = 10;  % 松弛因子
fprintf('Update start, t = %6.3f\n', t)



% 参数设计
kesi2=zeros(Nx2+Nu2,1);
kesi2(1)=u(1)-targetx(1,1);%u(1)==X(1);
kesi2(2)=u(2)-targety(1,1);%u(2)==X(2)
kesi2(3)=t_d-targetp(1,1);%u(3)==X(3)
kesi2(4)=U2(1);
kesi2(5)=U2(2);
fprintf('Update start,u(1)=%4.2f\n',U2(1))
fprintf('Update start,u(2)=%4.2f\n',U2(2))
%矩阵初始化
u_piao2=zeros(Nx2,Nu2);%%%%%%后边用来装优化计算结果（控制增量）
Q=1000*eye(Nx2*Np2,Nx2*Np2);
R=50*eye(Nu2*Nc2);
a2=cell(1,Np2);
b2=cell(1,Np2);
for i=1:1:Np2
    a2{1,i}=[1 0 -targetv(1,i)*sin(targetp(1,i))*T1;
            0 1 targetv(1,i)*cos(targetp(1,i))*T1;
            0 0 1;];%[Nx1 Nx1]
    b2{1,i}= [cos(targetp(1,i))*T1 0;
             sin(targetp(1,i))*T1 0;
             tan(targett(1,i))*T1/L targetv(1,i)*T1/(cos(targett(1,i))^2);];%[Nx1 Nu1]
end
% 对应（4.6）中的参数
A=cell(1,Np2);
B=cell(1,Np2);
for i=1:1:Np2
    A_cell=cell(2,2);
    B_cell=cell(2,1);
    A_cell{1,1}=cell2mat(a2(1,i));
    A_cell{1,2}=cell2mat(b2(1,i));
    A_cell{2,1}=zeros(Nu2,Nx2);
    A_cell{2,2}=eye(Nu2);
    B_cell{1,1}=cell2mat(b2(1,i));
    B_cell{2,1}=eye(Nu2);
    A{1,i}=cell2mat(A_cell);%[Nx1+Nu1 Nx1+Nu1]
    B{1,i}=cell2mat(B_cell);%[Nx1+Nu1 Nu1]
end
C=[1 0 0 0 0;0 1 0 0 0;0 0 1 0 0;];
PHI_cell=cell(Np2,1);
THETA_cell=cell(Np2,Nc2);
for j=1:1:Np2
    if j==1
        PHI_cell{j,1}=C*A{1,j};
    else
        AA=A{1,j};
        for i=j-1:-1:1
            AA=AA*A{1,i};
        end
        PHI_cell{j,1}=C*AA;
    end
    for k=1:1:Nc2
        
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
                THETA_cell{j,k}=zeros(Nx2,Nu2);
            end
        end
        
    end
end
PHI=cell2mat(PHI_cell);%size(PHI)=[Nx1*Np1 Nx1+Nu1]
THETA=cell2mat(THETA_cell);%size(THETA)=[Nx1*Np1 Nu1*Nc1]
H_cell=cell(2,2);
H_cell{1,1}=THETA'*Q*THETA+R;
H_cell{1,2}=zeros(Nu2*Nc2,1);
H_cell{2,1}=zeros(1,Nu2*Nc2);
H_cell{2,2}=Row;
H=cell2mat(H_cell);

error=PHI*kesi2;
f_cell=cell(1,2);
f_cell{1,1}=2*error'*Q*THETA;
f_cell{1,2}=0;
%f=cell2mat(f_cell)';
f=cell2mat(f_cell);
% %以下为约束生成区域
%不等式约束
A_t=zeros(Nc2,Nc2);%见falcone论文P181
for p=1:1:Nc2
    for q=1:1:Nc2
        if q<=p
            A_t(p,q)=1;
        else
            A_t(p,q)=0;
        end
    end
end
A_l=kron(A_t,eye(Nu2));%对应于falcone论文约束处理的矩阵A，求克罗内克积
Ut=kron(ones(Nc2,1),U2);%此处感觉论文里的克罗内科积有问题，暂时交换顺序
% % % umin=[-0.2;-0.54;];%维数与控制变量的个数相同
% % % umax=[0.2;0.332;];


umin=[-0.2;-0.54;];%维数与控制变量的个数相同
umax=[0.2;0.332;];
delta_umin=[-0.05;-0.0082;];%delta_umin=[0.05;-0.0082;];原代码有错，速度变化下界没加负号
delta_umax=[0.05;0.0082];


Umin=kron(ones(Nc2,1),umin);
Umax=kron(ones(Nc2,1),umax);
A_cons_cell={A_l zeros(Nu2*Nc2,1);-A_l zeros(Nu2*Nc2,1)};
b_cons_cell={Umax-Ut;-Umin+Ut};
A_cons=cell2mat(A_cons_cell);%(求解方程)状态量不等式约束增益矩阵，转换为绝对值的取值范围
b_cons=cell2mat(b_cons_cell);%(求解方程)状态量不等式约束的取值
%状态量约束
M=10;
delta_Umin=kron(ones(Nc2,1),delta_umin);
delta_Umax=kron(ones(Nc2,1),delta_umax);
lb=[delta_Umin;0];%(求解方程)状态量下界，包含控制时域内控制增量和松弛因子
ub=[delta_Umax;M];%(求解方程)状态量上界，包含控制时域内控制增量和松弛因子

% %开始求解过程
%options = optimset('Algorithm','active-set');新版quadprog不能用有效集法，这里选内点法
options = optimset('Algorithm','interior-point-convex');
% [X,fval,exitflag]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub,[],options);
[X,fval,exitflag]=quadprog(H,f,A_cons,b_cons,[],[],[],[],[],options);


% %判断是否到达终点
% dis=norm([u(1)-20 u(2)-62]);
% if(abs(dis)<0.2)
%     f=1;
% end 


% %计算输出
u_piao2(1)=X(1);
u_piao2(2)=X(2);
U2(1)=kesi2(4)+u_piao2(1);
U2(2)=kesi2(5)+u_piao2(2);
u_real(1)=U2(1)+targetv(1,1);
u_real(2)=U2(2)+targett(1,1);
u_real(3)=point22;

% 
% %判断是否到达终点，停止
% if f==1
%     u_real(1)=0;
%     u_real(2)=0;
%     u_real(3)=0;
% end 


sys=u_real;
%toc
% 输出结束