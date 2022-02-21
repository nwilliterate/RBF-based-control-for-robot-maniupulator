% Copyright (C) 2022 All rights reserved.
% Authors:     Seonghyeon Jo <cpsc.seonghyeon@gmail.com>
%
% Date:         Feb, 21, 2022
% Last Updated: Feb, 21, 2022
% 
% -------------------------------------------------
% RBF observer 
% two link manipulator
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

% parameter of neural network
variance = 50;
Node = 256;
W1 = zeros(Node, 2);
Mu1 = [-1:2/(Node-1):1];
k1=0.01;k2=0.1;
F1 = 50000*eye(Node);

% paramter observer
global K bar D 
K = diag([25 25]);

% reference trajectory
t1 = 0.25;
t2 = 0.5;
ref_q(1,:)=(10*sin(t1*pi*t)+10*sin(t2*pi*t))/180*pi;
ref_q(2,:)=(10*sin(t1*pi*t)+10*sin(t2*pi*t))/180*pi;
ref_qd(1,:)=(10*t1*pi*cos(t1*pi*t)+10*t2*pi*cos(t2*pi*t))/180*pi;
ref_qd(2,:)=(10*t1*pi*cos(t1*pi*t)+10*t2*pi*cos(t2*pi*t))/180*pi;
ref_qdd(1,:)=-(10*t1^2*pi^2*sin(t1*pi*t)-10*t1^2*pi^2*sin(t2*pi*t))/180*pi;
ref_qdd(2,:)=-(10*t1^2*pi^2*sin(t1*pi*t)-10*t1^2*pi^2*sin(t2*pi*t))/180*pi;

% disturbance
f_dis(1,:) = 1.5*sin(t)+2;
f_dis(2,:) = 2.5*cos(t)+0.5;

% intial state
x(:,1) = [pi/4; -pi/4; 0; 0;];
x_hat(:,1) = [0; 0; 0; 0;];

H_bar = zeros(Node, 1);
for i=1:sample_size
    x1 = x(1:2,i);
    x2 = x(3:4,i);
    xhat1 = x_hat(1:2,i);
    xhat2 = x_hat(3:4,i);
    
    bar = 0.4;
    M = get_MassMatrix(xhat1) * bar;
    Cq = get_CoriolisVector(xhat1, xhat2) *bar;
    G = get_GravityVector(xhat1) * bar;
    F = get_FrictionVector(xhat2) * bar;
   
    
    Z = [xhat1' xhat2'];
    H = zeros(Node,1);
    for j=1:Node
        H(j) = exp(-norm(Z-Mu1(:,j))^2/(variance));
    end
    fx_hat(:, i) = (W1'*H_bar);
    
    y_tilde = x1-xhat1;
    dw1 = F1*H_bar*y_tilde'- k1*F1*W1;
    H_bar = H - .5*H_bar;
    
    D = 75;
    U(:,i) = ones(2,1)*(2*sin(2*t(i))+ 3*cos(10*t(i)));
    obs_u = [x1' fx_hat(:, i)' U(:,i)'];
    
    if i ~= sample_size
        x(:,i+1) = rk(x(:,i), U(:,i),sim_period);
        x_hat(:,i+1) = obs_rk(x_hat(:,i), obs_u, sim_period);
        
        W1 = W1 + dw1*0.001;
    end
end
RMSE = mean(abs(x(1:2,:)-x_hat(1:2,:)),2)

% plot
% figure 1 : q
figure(1)
tiledlayout(2,1,'TileSpacing','Compact','Padding','Compact');
set(gcf,'color','w');
for i=1:2
    ax = nexttile;
    plot(t, x(i,:),'-k','LineWidth',1.5')
    hold on
    plot(t, x_hat(i,:),'-.r','LineWidth',1.5');
%     plot(t, ref_q(i,:),'--b','LineWidth',1.5');
    hold off
    ylim([ax.YLim(1)-0.025  ax.YLim(2)+0.025])
    xlim([0 sample_size*0.001])
    xlabel('time(s)', 'FontSize', 10)
    ylabel("q_{"+i+ "}(rad)", 'FontSize', 10);
    grid on;
    legend('q', 'qhat')
end
% saveas(gcf,"fig\q_result.png");

% figure 2 : input
figure(2)
tiledlayout(2,1,'TileSpacing','Compact','Padding','Compact');
set(gcf,'color','w');
for i=1:2
    ax = nexttile;
    plot(t, U(i,:),'-k','LineWidth',1.5');
    ylim([ax.YLim(1)-0.025  ax.YLim(2)+0.025])
    xlim([0 sample_size*0.001])
    xlabel('time(s)', 'FontSize', 10)
    ylabel("u_{"+i+ "}(Nm)", 'FontSize', 10);
    grid on;
    legend('u')
end
% plot
% figure 3
figure(3)
tiledlayout(2,1,'TileSpacing','Compact','Padding','Compact');
set(gcf,'color','w');
for i=1:2
    ax = nexttile;
    plot(t, fx_hat(i,:),'-k','LineWidth',1.5')
    ylim([ax.YLim(1)-0.025  ax.YLim(2)+0.025])
    xlim([0 sample_size*0.001])
    xlabel('time(s)', 'FontSize', 10)
    ylabel("nn_{"+i+ "}(rad)", 'FontSize', 10);
    grid on;
    legend('e')
end

% plot
% figure 4: error
figure(4)
tiledlayout(2,1,'TileSpacing','Compact','Padding','Compact');
set(gcf,'color','w');
for i=1:2
    ax = nexttile;
    plot(t, x(i,:)-x_hat(i,:),'-k','LineWidth',1.5')
    ylim([ax.YLim(1)-0.025  ax.YLim(2)+0.025])
    xlim([0 sample_size*0.001])
    xlabel('time(s)', 'FontSize', 10)
    ylabel("q_{"+i+ "}(rad)", 'FontSize', 10);
    grid on;
    legend('e')
end