%仿真+一主一从画图用的程序
clear all;
sim('simulation');  %仿真三农机
   

%    xlim([-10,30]);
%    ylim([-15,70]);

% h = animatedline;
% h1 = animatedline;
% h2 = animatedline;
 
x = simout.Data(:,1);    %主车数据
y = simout.Data(:,2);
x1 = simout1.Data(:,1);   %1号随车数据
y1 = simout1.Data(:,2);
% x2 = simout2.Data(:,1);   %2号随车数据
% y2 = simout2.Data(:,2);


figure(3);                  %动态画图
plot(x,y,'-b',x1,y1,'-r');
hold on;
for i=1:length(x)
    hold on
    t1=x(i);
    m1=y(i);
    t2=x1(i);
    m2=y1(i);
%     t3=x2(i);
%     m3=y2(i);
                %动态画图
%     t1=[x(i),x(i+1)];
%     m1=[y(i),y(i+1)];
%     t2=[x1(i),x1(i+1)];
%     m2=[y1(i),y1(i+1)];
%     t3=[x2(i),x2(i+1)];
%     m3=[y2(i),y2(i+1)];
    axis([-100,400,-50,200]);
%     
    Lead_Car=plot(t1,m1,'Color','blue','Marker','o','LineWidth',0.5);
    Follow_Car1=plot(t2,m2,'Color','red','Marker','s','LineWidth',0.5);
%     Follow_Car2=plot(t3,m3,'Color','green','Marker','v','LineWidth',0.5);
    pause(0.0005);
    if i < length(x)
       delete(Lead_Car)
       delete(Follow_Car1)
%        delete(Follow_Car2)
    end
%     saveas(gcf, 'aaa.pdf') 
end