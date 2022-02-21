% Copyright (C) 2022 All rights reserved.
% Authors:     Seonghyeon Jo <cpsc.seonghyeon@gmail.com>
%
% Date:         Feb, 21, 2022
% Last Updated: Feb, 21, 2022
% 
% -------------------------------------------------
% RBF observer 
% Single-link manipulator 
% -------------------------------------------------
%
% the following code has been tested on matlab 2021a
%%
clc; clear all;
addpath(genpath('.'));

% simulation setup
sim_period = 0.001;
t = 0:sim_period:50;
sample_size = size(t, 2);

% % reference trajectory
xd(1,:)=sin(t);
xd(2,:)=cos(t);
xd(3,:)=-sin(t);

% parameter of neural network
variance = 50;
Node = 7;
W1 = zeros(Node, 1);
W2 = zeros(Node, 1);
Mu1 = [-1:2/(Node-1):1];
Mu2 = [-1:2/(Node-1):1]*2;
Mu = [Mu1; Mu2];
k1=0.1;k2=0.1;
F1= 500000*eye(7);
F2= 50000*eye(7);

% intial state
x(:,1) = [0; 0.5];
x_hat(:,1) = [0.1; 0];

global D K1 K2

H_bar = zeros(Node, 1);
% dH = zeros(Node, 1);
for i=1:sample_size
    x1 = x(1,i);
    x2 = x(2,i);
    
    % ann
    y_tilde = x1 - x_hat(1,i);
    H = zeros(Node,1);
    for j=1:Node
        H(j) = exp(-norm(x(:,i)-Mu(:,j))^2/(variance));
    end
    fx_hat = (W1'*H_bar) + 0.0001;
    gx_hat = (W2'*H_bar) + 0.0001;
    
    U(1,i) = sin(2*t(i))+ cos(20*t(i));
    
    D = 2;
    K1 = 400;
    K2 = 2000;
    
    u_obs = [x1 U(1,i) fx_hat gx_hat];
    
    dw1 = F1*H_bar*y_tilde - k1*F1*abs(y_tilde)*W1;
    dw2 = F2*H_bar*y_tilde*U(1,i) - k2*F2*abs(y_tilde)*W2;
    H_bar = H - .5*H_bar;
    
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
    plot(t, x_hat(i,:),'-.r','LineWidth',1.5');
    hold off;
    ylim([ax.YLim(1)-0.025  ax.YLim(2)+0.025])
    xlabel('time[s]', 'FontSize', 10)
    ylabel("nn_{"+i+ "}[rad]", 'FontSize', 10);
    grid on;
    legend('x','hat x')
end