% Copyright (C) 2022 All rights reserved.
% Authors:     Seonghyeon Jo <cpsc.seonghyeon@gmail.com>
%
% Date:         Feb, 21, 2022
% Last Updated: Feb, 21, 2022
% 
% -------------------------------------------------
% RBF observer based sllding mode control
% Single-link manipulator 
% -------------------------------------------------
%
% the following code has been tested on matlab 2021a
%%
clc; clear all;
addpath(genpath('.'));

% simulation setup
sim_period = 0.001;
t = 0:sim_period:20;
sample_size = size(t, 2);

% % reference trajectory
xd(1,:)=sin(t);
xd(2,:)=cos(t);
xd(3,:)=-sin(t);

% parameter of neural network
variance = 25;
Node = 7;
W1 = zeros(Node, 1);
W2 = zeros(Node, 1);
Mu1 = [-1:2/(Node-1):1]*1/3;
Mu2 = [-1:2/(Node-1):1]*1/3;
Mu = [Mu1; Mu2];
k1=0.01;k2=0.01;
F1= 500*eye(Node);
F2= .5*eye(Node);

% intial state
x(:,1) = [0.2; 0];
x_hat(:,1) = [0.1; 0];

global D K1 K2

H_bar = zeros(Node, 1);
for i=1:sample_size
    x1 = x(1,i);
    x2 = x(2,i);
    
    x_hat1 = x_hat(1,i);
    x_hat2 = x_hat(2,i);
    
    % model
    m=1;l=1;M=0.5;g=9.8;
    fx=-0.5*m*g*l*sin(x1)/M;
    gx=1/M;
    
    % ann
    y_tilde = x1 - x_hat(1,i);
    H = zeros(Node,1);
    for j=1:Node
        H(j) = exp(-norm(x(:,i)-Mu(:,j))^2/(variance));
    end
    fx_hat = (W1'*H_bar);
    gx_hat = (W2'*H_bar)+1;
    
    nu = 10; %0.10;
    c = 20;
    
    % err
    e1 = x_hat1 - xd(1,i);
    e2 = x_hat2 - xd(2,i);
    
    s = c*e1 + e2;
    
    D=1.5;
    v=-D*sign(y_tilde);
    
    K1 = 400;
    K2 = 8000;
    u =(1/(gx_hat))*(-c*(x_hat2+K1*(x1- x_hat1)-xd(2,i)) -fx_hat + v - K2*(x1 - x_hat1)+xd(3,i)-nu*sign(s));
    
    U(1,i) = u;
    
    u_obs = [x1 U(1,i) fx_hat gx_hat];
    
    dw1 = F1*H_bar*y_tilde - k1*F1*abs(y_tilde)*W1;
    dw2 = F2*H_bar*y_tilde*U(1,i) - k2*F2*abs(y_tilde)*W2;
    H_bar = H - 0.5*H_bar;
    
    if i ~= sample_size
        x(:,i+1) = rk(x(:,i),U(1,i),sim_period);
        x_hat(:,i+1) = obs_rk(x_hat(:,i),u_obs,sim_period);
        W1 = W1 + dw1*0.001;
        W2 = W2 + dw2*0.001;
    end
end
fig = figure(1);
tiledlayout(2,1,'TileSpacing','Compact','Padding','Compact');
set(gcf,'color','w');
for i=1:2
    ax = nexttile;
    plot(t, x(i,:) ,'-k','LineWidth',1.5');
    hold on;
    plot(t, xd(i,:) ,'--b','LineWidth',1.5');
    plot(t, x_hat(i,:),'-.r','LineWidth',1.5');
    hold off;
    ylim([ax.YLim(1)-0.025  ax.YLim(2)+0.025])
    xlabel('time[s]', 'FontSize', 12)
    if i==1
        ylabel("q [rad]", 'FontSize', 12);
    else
        ylabel("dq [rad / s]", 'FontSize', 12);
    end
    grid on;
    legend('x', 'xd', 'xhat')
end