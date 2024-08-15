%�ӻ�
function[sys, x0, str, ts] = MY_MPCController(t, x, u, flag)
switch flag
    case 0   %��ʼ��
        [sys, x0, str, ts] = mdlInitializeSizes;  %��ʼ��
    case 2  %������ɢ״̬
        sys = mdlUpdates(t, x, u);
    case 3  %�������
        sys = mdlOutputs(t, x, u);
    case {1, 4, 9, 10}  % Unused flags
        sys = [];
    otherwise  %δ֪��flag
        error(['unhandled flag =' ,num2str(flag)]);  %Error handling
end
% s�������������

% ��ʼ���Ӻ���
function[sys, x0, str, ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates = 0;  % ����״̬���ĸ���
sizes.NumDiscStates = 3;  % ��ɢ״̬���ĸ���
sizes.NumOutputs = 2;  % ������ĸ���
sizes.NumInputs = 4;  %�������ĸ���
sizes.DirFeedthrough = 1;  % ����D�ǿգ� ֱ�ӹ�ͨ��־
sizes.NumSampleTimes = 1;  % ����ʱ��ĸ���
sys = simsizes(sizes);
x0 = [0; 0; 0];  % ״̬����ʼ��



global T1 L;
T1=0.1;%��������������
L=2.05;%ũ���������
global Nx Nu Np Nc;
Nx=3;%״̬���ĸ���
Nu=2;%�������ĸ���
Np=5;%Ԥ�ⲽ��
Nc=5;%���Ʋ���
global point2 count ;
count=0;%����������������
% point_flag=0;%�����켣���μ����־
point2=1;%%�������ʼ��
% x0=zeros(1,Nc*2);%%�Ż���ʼֵ��ʼ��

global r ;


%%%������ͣ���ĵ��ݶ�Ϊ(0,-8)
%%%����A���B������
ax=0;
ay=-8;
bx=300;
by=-8;





%%%����ũ��ת��뾶
R=5;
%%%���ù����оࣨ����ڻ��������ũ��ת��뾶��
workingdistance=11;
%%%����ũ�������͵�ͷ��ʻ�ٶȣ�����/Сʱ��
workingvelocity=10;
turningvelocity=5;
%%%���õؿ���
breadth=100;
%%%����·�����������ڣ��룩
global T
T=0.01;
%%%��ø�������
row=floor(breadth/workingdistance);
%%%���ó�ʼת��0Ϊ��ʱ�뷽��1Ϊ˳ʱ�뷽��
initialturning=0;
%%%����AB�߼�������
abx=bx-ax;
aby=by-ay;
direction=[];
%%%ȷ��AB�߷��򣨴�A��B��
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
%%%ȷ��AB�߳��ȣ�ֱ�߶ι������ȣ�
distance=sqrt(abx^2+aby^2);
global quadrant turningstraightlinepoint workingpathpoint;
%%%���㹤��ֱ�ߡ�ת�����ķ�֮һԲ��ת��ֱ�ߡ�ת����ķ�֮һԲ����·������,������ȡ��
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




%%%%%%��һ��ж����

%%%��һ��ж��������
unload_x1 = 10;
unload_y1 = 14;
direction_unload1 = atan((unload_y1 - ay)/(unload_x1 - ax));
%%%��һ�����ص�����
unloadback_x1 = 10;
unloadback_y1 = 25;
direction_unloadback1 = atan((unloadback_y1 - ay)/(unloadback_x1 - ax));
%%%��ͣ���㵽��һ��ж�����ֱ�߾���
straight_unload1_distance = sqrt((unload_x1 - ax)^2 + (unload_y1 - ay)^2);
%%%�ӵ�һ�����ص㵽ͣ�����ֱ�߾���
straight_unloadback1_distance = sqrt((unloadback_x1 - ax)^2 + (unloadback_y1 - ay)^2);
%%%��һ��ж������ʣ��ֱ��·������
workingafter_unload1_distance = distance - unload_x1;
%%%��һ�����ص�ǰ��ʣ��ֱ�߾���
workingbefore_unloadback1_distance = distance - unloadback_x1;


%%%�Ѿ���ת���ɹ�������
unload1point = floor(straight_unload1_distance/((workingvelocity*1000/3600)/(1/T)));
unloadback1point = floor(straight_unloadback1_distance/((workingvelocity*1000/3600)/(1/T)));

workingpathpoint_afterunload1 = floor(workingafter_unload1_distance/((workingvelocity*1000/3600)/(1/T)));
workingpathpoint_beforeunloadback1 = floor(workingbefore_unloadback1_distance/((workingvelocity*1000/3600)/(1/T)));

%%%%%%��һ��ж�������ý���



%%%%%%�ڶ���ж����

%%%�ڶ���ж��������
unload_x2 = 90;
unload_y2 = 47;
direction_unload2 = atan((unload_y2 - ay)/(unload_x2 - ax));
%%%�ڶ������ص�����
unloadback_x2 = 250;
unloadback_y2 = 58;
direction_unloadback2 = atan((unloadback_y2 - ay)/(unloadback_x2 - ax));
%%%��ͣ���㵽�ڶ���ж�����ֱ�߾���
straight_unload2_distance = sqrt((unload_x2 - ax)^2 + (unload_y2 - ay)^2);
%%%�ӵڶ������ص㵽ͣ�����ֱ�߾���
straight_unloadback2_distance = sqrt((unloadback_x2 - ax)^2 + (unloadback_y2 - ay)^2);
%%%�ڶ���ж������ʣ��ֱ��·������
workingafter_unload2_distance = distance - unload_x2;
%%%�ڶ������ص�ǰ��ʣ��ֱ�߾���
workingbefore_unloadback2_distance = distance - unloadback_x2;


%%%�Ѿ���ת���ɹ�������
unload2point = floor(straight_unload2_distance/((workingvelocity*1000/3600)/(1/T)));
unloadback2point = floor(straight_unloadback2_distance/((workingvelocity*1000/3600)/(1/T))) - 50;

workingpathpoint_afterunload2 = floor(workingafter_unload2_distance/((workingvelocity*1000/3600)/(1/T)));
workingpathpoint_beforeunloadback2 = floor(workingbefore_unloadback2_distance/((workingvelocity*1000/3600)/(1/T)));

%%%%%%�ڶ���ж�������ý���


%%%%%����ж��������
unloadpoint = [unload1point,unload2point];
direction_unload = [direction_unload1,direction_unload2];
workingpathpoint_afterunload = [workingpathpoint_afterunload1,workingpathpoint_afterunload2];
workingafter_unload_distance = [workingafter_unload1_distance,workingafter_unload2_distance];
workingpathpoint_beforeunloadback = [workingpathpoint_beforeunloadback1,workingpathpoint_beforeunloadback2];
direction_unloadback = [direction_unloadback1,direction_unloadback2];
unloadbackpoint = [unloadback1point,unloadback2point];





%%%����·��
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
            r(point+j,2) = xstart;  %x����
            r(point+j,3) = ystart;  %y����
            r(point+j,4) = direction_unload(i); %һ��ʼ�ķ���
            r(point+j,5) = 0;       %����ת��
            r(point+j,6) = workingvelocity*1000/3600;   %�����ٶ�
            r(point+j,7) = 3;%ֱ��/ת��/����/���ر�־     ֱ��0��ת��1��ת��ֱ��2������3������4
            r(point+j,8) = (workingvelocity*1000/3600)/(1/T);%ǰ��ж����ֱ�߶����켣���ľ���
        else
            r(point+j,1) = point + j;
            r(point+j,2) = r(point+j-1,2)+(workingvelocity*1000/3600)/(1/T)*cos(direction_unload(i));%x����
            r(point+j,3) = r(point+j-1,3)+(workingvelocity*1000/3600)/(1/T)*sin(direction_unload(i));%y����
            r(point+j,4) = direction_unload(i); %һ��ʼ�ķ���(����ֱ��һֱǰ��)
            r(point+j,5) = 0;       %����ת��
            if j<=acceleration_distance_count
                r(point+j,6)=r(point+j-1,6)+add_velocity;%�����ٶ�
            elseif j>workingpathpoint-acceleration_distance_count
                r(point+j,6)=r(point+j-1,6)-add_velocity;%�����ٶ�
            else
                r(point+j,6)=workingvelocity*1000/3600;
            end
            r(point+j,7) = 3;
            r(point+j,8) = (workingvelocity*1000/3600)/(1/T);%ǰ��ж����ֱ�߶����켣���ľ���
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

%                 if turningflag==0%%��0Ϊ��ʱ�뷽��1Ϊ˳ʱ�뷽��
%                     fistart=fistart+pi/2;
%                 end
%                 if turningflag==1%%��0Ϊ��ʱ�뷽��1Ϊ˳ʱ�뷽��
%                     fistart=fistart-pi/2;
%                 end
%                 if turningflag==0
%                     turningflag=1;
%                 elseif turningflag==1
%                     turningflag=0;
%                 end
            end
            r(point+j,1)=point+j;
            r(point+j,2)=xstart;%x����
            r(point+j,3)=ystart;%y����
            r(point+j,4)=fistart;
            r(point+j,5)=0;%����ת��
            r(point+j,6)=turningvelocity*1000/3600;%�����ٶ�
            r(point+j,7)=0;%ֱ��/ת���־
            r(point+j,8)=(workingvelocity*1000/3600)/(1/T);%����ֱ�߶����켣���ľ���
        else
            r(point+j,1)=point+j;
            r(point+j,2)=r(point+j-1,2)+(workingvelocity*1000/3600)/(1/T)*cos(fistart);%x����
            r(point+j,3)=r(point+j-1,3)+(workingvelocity*1000/3600)/(1/T)*sin(fistart);%y����
            r(point+j,4)=fistart;
            r(point+j,5)=0;%����ת��
            if j<=acceleration_distance_count
                r(point+j,6)=r(point+j-1,6)+add_velocity;%�����ٶ�
            elseif j>workingpathpoint_afterunload(i)-acceleration_distance_count
                r(point+j,6)=r(point+j-1,6)-add_velocity;%�����ٶ�
            else
                r(point+j,6)=workingvelocity*1000/3600;
            end
            r(point+j,7)=0;
            r(point+j,8)=(workingvelocity*1000/3600)/(1/T);%����ֱ�߶����켣���ľ���
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
            r(point+j,2)=xstart;%x����
            r(point+j,3)=ystart;%y����
            r(point+j,4)=fistart;
            %                 r(point+j,5)=-atan(L/R);%����ת��
            r(point+j,6)=turningvelocity*1000/3600;%�����ٶ�
            r(point+j,7)=1;
            r(point+j,8)=(turningvelocity*1000/3600)/(1/T);%ת�䴦���켣���Ļ���
            if turningflag==0%%��0Ϊ��ʱ�뷽��1Ϊ˳ʱ�뷽��
                r(point+j,5)=atan(L/R);%����ת��
                xo=xstart-R*sin(fistart);%%ת��Բ�ĵ�x����
                yo=ystart+R*cos(fistart);%%ת��Բ�ĵ�y����
            end
            if turningflag==1%%��0Ϊ��ʱ�뷽��1Ϊ˳ʱ�뷽��
                r(point+j,5)=-atan(L/R);%����ת��
                xo=xstart+R*sin(fistart);%%ת��Բ�ĵ�x����
                yo=ystart-R*cos(fistart);%%ת��Բ�ĵ�y����
            end
        end
        if turningflag==0%%��0Ϊ��ʱ�뷽��1Ϊ˳ʱ�뷽��
            r(point+j,1)=point+j;
            r(point+j,2)=xo+R*sin(r(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
            r(point+j,3)=yo-R*cos(r(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
            r(point+j,4)=r(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T)));
            r(point+j,5)=atan(L/R);%����ת��
            r(point+j,6)=turningvelocity*1000/3600;%�����ٶ�
            r(point+j,7)=1;
            r(point+j,8)=(turningvelocity*1000/3600)/(1/T);%ת�䴦���켣���Ļ���
        end
        if turningflag==1%%��0Ϊ��ʱ�뷽��1Ϊ˳ʱ�뷽��
            r(point+j,1)=point+j;
            r(point+j,2)=xo-R*sin(r(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
            r(point+j,3)=yo+R*cos(r(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
            r(point+j,4)=r(point+j-1,4)-2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T)));
            r(point+j,5)=-atan(L/R);%����ת��
            r(point+j,6)=turningvelocity*1000/3600;%�����ٶ�
            r(point+j,7)=1;
            r(point+j,8)=(turningvelocity*1000/3600)/(1/T);%ת�䴦���켣���Ļ���
        end
    end
    %         plot(r(:,2),r(:,3))
    point=point+quadrant;

    for j=1:1:turningstraightlinepoint
        if j==1
            xstart=xo+R*cos(fistart);
            ystart=yo+R*sin(fistart);
            if turningflag==0%%��0Ϊ��ʱ�뷽��1Ϊ˳ʱ�뷽��
                fistart=fistart+pi/2;
            end
            if turningflag==1%%��0Ϊ��ʱ�뷽��1Ϊ˳ʱ�뷽��
                fistart=fistart-pi/2;
            end
            r(point+j,1)=point+j;
            r(point+j,2)=xstart;
            r(point+j,3)=ystart;
            r(point+j,4)=fistart;
            r(point+j,5)=0;%����ת��
            r(point+j,6)=turningvelocity*1000/3600;%�����ٶ�
            r(point+j,7)=2;
            r(point+j,8)=(turningvelocity*1000/3600)/(1/T);%ת��ֱ�ߴ����켣���ľ���
        else
            r(point+j,1)=point+j;
            r(point+j,2)=r(point+j-1,2)+(turningvelocity*1000/3600)/(1/T)*cos(fistart);%x����
            r(point+j,3)=r(point+j-1,3)+(turningvelocity*1000/3600)/(1/T)*sin(fistart);%y����
            r(point+j,4)=fistart;
            r(point+j,5)=0;%����ת��
            r(point+j,6)=turningvelocity*1000/3600;%�����ٶ�
            r(point+j,7)=2;
            r(point+j,8)=(turningvelocity*1000/3600)/(1/T);%ת��ֱ�ߴ����켣���ľ���
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
            %                 r(point+j,5)=-atan(L/R);%����ת��
            r(point+j,6)=turningvelocity*1000/3600;%�����ٶ�
            r(point+j,7)=1;
            r(point+j,8)=(turningvelocity*1000/3600)/(1/T);%ת�䴦���켣���Ļ���
            if turningflag==0%%��0Ϊ��ʱ�뷽��1Ϊ˳ʱ�뷽��
                r(point+j,5)=atan(L/R);%����ת��
                xo=xstart-R*sin(fistart);%%ת��Բ�ĵ�x����
                yo=ystart+R*cos(fistart);%%ת��Բ�ĵ�y����
            end
            if turningflag==1%%��0Ϊ��ʱ�뷽��1Ϊ˳ʱ�뷽��
                r(point+j,5)=-atan(L/R);%����ת��
                xo=xstart+R*sin(fistart);%%ת��Բ�ĵ�x����
                yo=ystart-R*cos(fistart);%%ת��Բ�ĵ�y����
            end
        end
        if turningflag==0%%��0Ϊ��ʱ�뷽��1Ϊ˳ʱ�뷽��
            r(point+j,1)=point+j;
            r(point+j,2)=xo+R*sin(r(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
            r(point+j,3)=yo-R*cos(r(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
            r(point+j,4)=r(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T)));
            r(point+j,5)=atan(L/R);%����ת��
            r(point+j,6)=turningvelocity*1000/3600;%�����ٶ�
            r(point+j,7)=1;
            r(point+j,8)=(turningvelocity*1000/3600)/(1/T);%ת�䴦���켣���Ļ���
        end
        if turningflag==1%%��0Ϊ��ʱ�뷽��1Ϊ˳ʱ�뷽��
            r(point+j,1)=point+j;
            r(point+j,2)=xo-R*sin(r(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
            r(point+j,3)=yo+R*cos(r(point+j-1,4)+2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T))));
            r(point+j,4)=r(point+j-1,4)-2*pi/(2*pi*R/((turningvelocity*1000/3600)/(1/T)));
            r(point+j,5)=-atan(L/R);%����ת��
            r(point+j,6)=turningvelocity*1000/3600;%�����ٶ�
            r(point+j,7)=1;
            r(point+j,8)=(turningvelocity*1000/3600)/(1/T);%ת�䴦���켣���Ļ���
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
            %                 if turningflag==0%%��0Ϊ��ʱ�뷽��1Ϊ˳ʱ�뷽��
            %                     fistart=fistart+pi/2;
            %                 end
            %                 if turningflag==1%%��0Ϊ��ʱ�뷽��1Ϊ˳ʱ�뷽��
            %                     fistart=fistart-pi/2;
            %                 end
            %                 if turningflag==0
            %                     turningflag=1;
            %                 elseif turningflag==1
            %                     turningflag=0;
            %                 end
            %             end
            r(point+j,1)=point+j;
            r(point+j,2)=xstart;%x����
            r(point+j,3)=ystart;%y����
            r(point+j,4)=fistart;
            r(point+j,5)=0;%����ת��
            r(point+j,6)=turningvelocity*1000/3600;%�����ٶ�
            r(point+j,7)=0;%ֱ��/ת���־
            r(point+j,8)=(workingvelocity*1000/3600)/(1/T);%����ֱ�߶����켣���ľ���
        else
            r(point+j,1)=point+j;
            r(point+j,2)=r(point+j-1,2)+(workingvelocity*1000/3600)/(1/T)*cos(fistart);%x����
            r(point+j,3)=r(point+j-1,3)+(workingvelocity*1000/3600)/(1/T)*sin(fistart);%y����
            r(point+j,4)=fistart;
            r(point+j,5)=0;%����ת��
            if j<=acceleration_distance_count
                r(point+j,6)=r(point+j-1,6)+add_velocity;%�����ٶ�
            elseif j>workingpathpoint_beforeunloadback(i)-acceleration_distance_count
                r(point+j,6)=r(point+j-1,6)-add_velocity;%�����ٶ�
            else
                r(point+j,6)=workingvelocity*1000/3600;
            end
            r(point+j,7)=0;
            r(point+j,8)=(workingvelocity*1000/3600)/(1/T);%����ֱ�߶����켣���ľ���
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
            %                             if turningflag==0%%��0Ϊ��ʱ�뷽��1Ϊ˳ʱ�뷽��
            %                                 fistart=fistart+pi/2;
            %                             end
            %                             if turningflag==1%%��0Ϊ��ʱ�뷽��1Ϊ˳ʱ�뷽��
            %                                 fistart=fistart-pi/2;
            %                             end
            %                             if turningflag==0
            %                                 turningflag=1;
            %                             elseif turningflag==1
            %                                 turningflag=0;
            %                             end
            %                         end
            r(point+j,1)=point+j;
            r(point+j,2)=xstart;%x����
            r(point+j,3)=ystart;%y����
            r(point+j,4)=fistart;
            r(point+j,5)=0;%����ת��
            r(point+j,6)=workingvelocity*1000/3600;%�����ٶ�
            r(point+j,7)=4;%ֱ��/ת���־
            r(point+j,8)=(workingvelocity*1000/3600)/(1/T);%����ֱ�߶����켣���ľ���
        else
            r(point+j,1)=point+j;
            r(point+j,2)=r(point+j-1,2)+(workingvelocity*1000/3600)/(1/T)*cos(fistart);%x����
            r(point+j,3)=r(point+j-1,3)+(workingvelocity*1000/3600)/(1/T)*sin(fistart);%y����
            r(point+j,4)=fistart;
            r(point+j,5)=0;%����ת��
            if j<=acceleration_distance_count
                r(point+j,6)=r(point+j-1,6)+add_velocity;%�����ٶ�
            elseif j>unloadbackpoint(i)-acceleration_distance_count
                r(point+j,6)=r(point+j-1,6)-add_velocity;%�����ٶ�
            else
                r(point+j,6)=workingvelocity*1000/3600;
            end
            r(point+j,7)=4;
            r(point+j,8)=(workingvelocity*1000/3600)/(1/T);%����ֱ�߶����켣���ľ���
        end
    end
    point=point+unloadbackpoint(i);

end



figure(3);
axis([-10,400,-50,200]);
hold on;
plot(r(:,2),r(:,3));



global U;%%%���������������ǲ�����������ʼ��
U=[0;0];
str = [];  % ����һ��str�վ���
ts = [0.05 0];  % sample time : [period, offset]
% ��ʼ���Ӻ�������









% ������ɢ��״̬���Ӻ���
function sys = mdlUpdates(t, x, u)
sys = x;
% ��ɢ��״̬���Ӻ�������











%��������Ӻ���
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
global U t_d targetp targetx targety targett targetv;%%�ֱ��Ӧ���ۺ����б���UU��tt_dd��uuu��targetpp��targetxx��targetyy
%     U(1)=u(4);
%     U(2)=u(5)*pi/180;


t_d=u(3);
targetp=zeros(1,Np);%%���������켣�㣨����
targetx=zeros(1,Np);%%���������켣�㣨x���꣩
targety=zeros(1,Np);%%���������켣�㣨y���꣩
targett=zeros(1,Np);%%���������켣�㣨ת�ǣ�
targetv=zeros(1,Np);%%���������켣�㣨�ٶȣ�




global point2;
global point21;     %%������point2


% %%%%%%������10m�Ĵ���
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



%%%%%%%%�������

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





% % ���ת����㷨��ȡ��ע�ͺ󼴿��ã�����������ôӻ���ֱ�߸���������������
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


Row = 10;  % �ɳ�����
fprintf('Update start, t = %6.3f\n', t)



% �������
kesi=zeros(Nx+Nu,1);
kesi(1)=u(1)-targetx(1,1);%u(1)==X(1);
kesi(2)=u(2)-targety(1,1);%u(2)==X(2)
kesi(3)=t_d-targetp(1,1);%u(3)==X(3)
kesi(4)=U(1);
kesi(5)=U(2);
fprintf('Update start,u(1)=%4.2f\n',U(1))
fprintf('Update start,u(2)=%4.2f\n',U(2))
%�����ʼ��
u_piao=zeros(Nx,Nu);%%%%%%�������װ�Ż�������������������
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
% ��Ӧ��4.6���еĲ���
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
% %����ΪԼ����������
%����ʽԼ��
A_t=zeros(Nc,Nc);%��falcone����P181
for p=1:1:Nc
    for q=1:1:Nc
        if q<=p
            A_t(p,q)=1;
        else
            A_t(p,q)=0;
        end
    end
end
A_l=kron(A_t,eye(Nu));%��Ӧ��falcone����Լ������ľ���A��������ڿ˻�
Ut=kron(ones(Nc,1),U);%�˴��о�������Ŀ����ڿƻ������⣬��ʱ����˳��
% % % umin=[-0.2;-0.54;];%ά������Ʊ����ĸ�����ͬ
% % % umax=[0.2;0.332;];

umin=[-2;-1;];%ά������Ʊ����ĸ�����ͬ
umax=[4;1;];
delta_umin=[-0.3;-0.03;];%delta_umin=[0.05;-0.0082;];ԭ�����д��ٶȱ仯�½�û�Ӹ���
delta_umax=[0.5;0.03];
% umin=[-0.2;-0.54;];%ά������Ʊ����ĸ�����ͬ
% umax=[0.2;0.332;];
% delta_umin=[-0.05;-0.0082;];%delta_umin=[0.05;-0.0082;];ԭ�����д��ٶȱ仯�½�û�Ӹ���
% delta_umax=[0.05;0.0082];

Umin=kron(ones(Nc,1),umin);
Umax=kron(ones(Nc,1),umax);
A_cons_cell={A_l zeros(Nu*Nc,1);-A_l zeros(Nu*Nc,1)};
b_cons_cell={Umax-Ut;-Umin+Ut};
A_cons=cell2mat(A_cons_cell);%(��ⷽ��)״̬������ʽԼ���������ת��Ϊ����ֵ��ȡֵ��Χ
b_cons=cell2mat(b_cons_cell);%(��ⷽ��)״̬������ʽԼ����ȡֵ
%״̬��Լ��
M=10;
delta_Umin=kron(ones(Nc,1),delta_umin);
delta_Umax=kron(ones(Nc,1),delta_umax);
lb=[delta_Umin;0];%(��ⷽ��)״̬���½磬��������ʱ���ڿ����������ɳ�����
ub=[delta_Umax;M];%(��ⷽ��)״̬���Ͻ磬��������ʱ���ڿ����������ɳ�����

% %��ʼ������
%options = optimset('Algorithm','active-set');�°�quadprog��������Ч����������ѡ�ڵ㷨
options = optimset('Algorithm','interior-point-convex');
% [X,fval,exitflag]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub,[],options);
[X,fval,exitflag]=quadprog(H,f,A_cons,b_cons,[],[],[],[],[],options);

% %�ж��Ƿ񵽴��յ�
% f=0;
% dis=norm([u(1)-20 u(2)-66]);
% if(abs(dis)<0.2)
%     f=1;
% end


% %�������
u_piao(1)=X(1);
u_piao(2)=X(2);
U(1)=kesi(4)+u_piao(1);
U(2)=kesi(5)+u_piao(2);
u_real(1)=U(1)+targetv(1,1);
u_real(2)=U(2)+targett(1,1);

% %�ж��Ƿ񵽴��յ㣬ֹͣ
% if f==1
%     u_real(1)=0;
%     u_real(2)=0;
% end

%%�ж��������Ƿ���Գ��������ո��δ������Ӧλ�ã���������һֱֹͣ

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
% �������