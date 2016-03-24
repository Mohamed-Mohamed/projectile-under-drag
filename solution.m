%% Coded by
% Mohamed Mohamed El-Sayed Atyya
% mohamed.atyya94@eng-st.cu.edu.eg



% This is solving a projectile under drag problem by RK1, RK4 and RK5
close all; clear all; clc;
%% I.C
syms x;
y0=1;  % initial y position
x0=1;   % initial x position
g=9.8;    % gravity
v0=50*sqrt(2);  % initial velocity
theta=45*pi/180;    % setting angle
cd=.02;      % drag coefficient
rho=1.225;      % air density
mp = 1;  % projectile mass
a=1;       % projectile area
%% numerical solution parameter
dt=1; % Time step
B=[0;0;-g;0];
X0=[v0*cos(theta);x0;v0*sin(theta);y0];  % dx/dt, x, dy/dt, y
order=4;
t_initial=0;
t_final=2000;
open=1;
%% RK4
sol(:,1)=X0;
m=2;
fig=figure();
menu = uicontrol('Parent',fig,'Style','popupmenu','String',{'Area';'theta';'V';'Exit'},'Units','centimeters' ,'Position',[17.5,0.25,3,0.5]);
slider = uicontrol('Parent',fig,'Style','slider','Units','centimeters' ,'Position',[0,0,10,0.5],'value',1,'SliderStep', [0.01,0.1] , 'min',1, 'max',100);
set(gcf,'color','w');
while open==1
    figure(fig);
    S = get(menu,'value');
    if S == 1
        clear sol;
        a=get(slider,'value')/100;
        sol(:,1)=[v0*cos(theta);x0;v0*sin(theta);y0];
    elseif S == 2
        clear sol;
        theta=get(slider,'value')*90/100;
        sol(:,1)=[v0*cos(theta);x0;v0*sin(theta);y0];
    elseif S == 3
        clear sol;
        v0=get(slider,'value');
        sol(:,1)=[v0*cos(theta);x0;v0*sin(theta);y0];
    elseif S==4
        open=0;
    end
    m=2;
    while sol(4,m-1) > 0
        A=[-rho*cd*cos(atan2(sol(3,m-1),sol(1,m-1)))*sol(1,m-1)*a/2/mp,0,-rho*cd*cos(atan2(sol(3,m-1),sol(1,m-1)))*sol(3,m-1)*a/2/mp,0;1,0,0,0;...
            -rho*cd*sin(atan2(sol(3,m-1),sol(1,m-1)))*sol(1,m-1)*a/2/mp,0,-rho*cd*sin(atan2(sol(3,m-1),sol(1,m-1)))*sol(3,m-1)*a/2/mp,0;0,0,1,0];
        [ S, t ] = RK4( A,B,sol(:,m-1),dt,dt*(m-1),dt*m,order );
        sol(:,m)=S(:,2);
        m=m+1;
    end
    plot(sol(2,:),sol(4,:),'red');
    legend(['Area = ' num2str(a) ', V = ' num2str(v0) ', \theta = ' num2str(theta)]);
    grid on; 
    xlabel('X','fontsize',18);
    ylabel('Y','fontsize',18);
    xlim([0,max(sol(2,:))]);
    ylim([0,max(sol(4,:))]);
    title('Projectile under drag','fontsize',18);
end
close all;