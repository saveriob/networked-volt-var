%	This code generates the simulations included in
%
%	S. Bolognani, R. Carli, G. Cavraro, and S. Zampieri
%   "On the need for communication for voltage regulation of power distribtuion grids"
%   under review.
%
%	This source code is distributed in the hope that it will be useful, but without any warranty.
%	We kidly request that you explicitly acknowledge the use of this code by citing the above mentioned paper.
%
%	tab width 4

clear variables
close all
clc

% Import MatPower library

matpower_startup;
define_constants;

% Load grid model

mpc = loadcase('case_ieee123_2compensators');
n = size(mpc.bus,1);        % size of the grid
c = [10; 32];               % set of microgenerators

%% Problem parameters

params.VMIN = 0.95;         % undervoltage limit
params.VMAX = 1.05;         % overvoltage limit

params.VMIN2 = 0.99;        % deadband lower limit
params.VMAX2 = 1.01;        % deadband upper limit

params.QMIN = [-1 -0.2];    % reactive power limits of the microgenerators

%% Simulation parameters

load pvproduction.mat
load demandprofile.mat

%% Uncontrolled

v_nocontrol = zeros(length(t),n);

for k = 1:length(t)
    
    % no reactive power compensation
    mpc.gen([1 2],QG) = [0; 0];
    
    % PV production
    mpc.gen([1 2],PG) = pvproduction(k,1:2)';
    
    % power demands
    mpc.bus(1:(n-1),PD) = p(k,:)';
    mpc.bus(1:(n-1),QD) = q(k,:)';
    
    % solve PF equations
    results = runpf(mpc, mpoption('VERBOSE', 0, 'OUT_ALL',0));
    
    v_nocontrol(k,:) = results.bus(:,VM);
    
end

%% Decentralized control (static)

v_static = zeros(length(t),n);
q_static = zeros(length(t),2);

for k = 1:length(t)
    
    % PV production
    mpc.gen([1 2],PG) = pvproduction(k,1:2)';
    
    % power demands
    mpc.bus(1:(n-1),PD) = p(k,:)';
    mpc.bus(1:(n-1),QD) = q(k,:)';
    
    % solve PF equations
    results = runpf(mpc, mpoption('VERBOSE', 0, 'OUT_ALL',0));
    
    v_static(k,:) = results.bus(:,VM);
    
    if k==length(t)
        break;
    end

    % decentralized reactive power compensation
    q_static(k+1,:) = localstatic(v_static(k,c),params);
    q_static(k+1,:) = max(q_static(k+1,:), params.QMIN);
    mpc.gen([1 2],QG) = q_static(k+1,:)';
    
end

%% Decentralized control (incremental)

v_incremental = zeros(length(t),n);
q_incremental = zeros(length(t),2);

for k = 1:length(t)
    
    % PV production
    mpc.gen([1 2],PG) = pvproduction(k,1:2)';
    
    % power demands
    mpc.bus(1:(n-1),PD) = p(k,:)';
    mpc.bus(1:(n-1),QD) = q(k,:)';
    
    % solve PF equations
    results = runpf(mpc, mpoption('VERBOSE', 0, 'OUT_ALL',0));
    
    v_incremental(k,:) = results.bus(:,VM);
    
    if k==length(t)
        break;
    end

    % decentralized reactive power compensation
    q_incremental(k+1,:) = q_incremental(k,:) - max(v_incremental(k,c)-params.VMAX,0);
    q_incremental(k+1,:) = max(q_incremental(k+1,:), params.QMIN);
    mpc.gen([1 2],QG) = q_incremental(k+1,:)';
    
end

%% Networked control

K = 10;                     % Sample time ratio for the inner feedback loop

v_networked = zeros(length(t),n);
q_networked = zeros(length(t),2);

lambda = zeros(length(t),2);
mu = zeros(1,2);
qhat = zeros(1,2);

G = gparameters(mpc,c);     % G gains

gamma = 1/(2*norm(G));      % gain of the inner feebdack loop
alpha = 10;                 % gain of the outer feedback loop

for k = 1:length(t)
    
    % PV production
    mpc.gen([1 2],PG) = pvproduction(k,1:2)';
    
    % power demands
    mpc.bus(1:(n-1),PD) = p(k,:)';
    mpc.bus(1:(n-1),QD) = q(k,:)';
    
    % solve PF equations
    results = runpf(mpc, mpoption('VERBOSE', 0, 'OUT_ALL',0));
    
    v_networked(k,:) = results.bus(:,VM);
    
    if k==length(t)
        break;
    end
    
    % networked reactive power compensation
    lambda(k+1,:) = max(0, lambda(k,:) + alpha*(v_networked(k,c) - params.VMAX));

    % inner loop
    for innerk = 1:K
        qhat = -lambda(k+1,:) + mu * G;
        mu = max(0, mu + gamma*(params.QMIN-qhat));
    end

    qhat = max(qhat, params.QMIN);
    q_networked(k+1,:) = qhat;
    mpc.gen([1 2],QG) = q_networked(k+1,:)';
    
end

%% Plots 

close all

colorone = [217,95,2]/255;
colortwo = [117,112,179]/255;
colorgrey = 200*[1,1,1]/255;

figure(1)
    hold on
    line([0 t(end)/60/60], [1 1], 'Color', 'black', 'LineStyle', '-')
    line([0 t(end)/60/60], [params.VMAX params.VMAX], 'Color', 'black', 'LineStyle', '--')
    plot(t/60/60,v_nocontrol, 'Color', colorgrey, 'LineWidth', 0.5)
    plot(t/60/60,v_nocontrol(:,c(1)), 'Color', colorone, 'LineWidth', 1)
    plot(t/60/60,v_nocontrol(:,c(2)), 'Color', colortwo, 'LineWidth', 1)
    xlabel('$t$ [h]')
    ylabel('$v$ [p.u.]')
    ylim([0.98 1.1])
    xlim([0 12])
    box on
    
figure(2)
    hold on
    line([0 t(end)/60/60], [params.VMAX params.VMAX], 'Color', 'black', 'LineStyle', '--')
    line([0 t(end)/60/60], [1 1], 'Color', 'black', 'LineStyle', '-')
    plot(t/60/60,v_static, 'Color', colorgrey, 'LineWidth', 0.5)
    plot(t/60/60,v_static(:,c(1)), 'Color', colorone, 'LineWidth', 1)
    plot(t/60/60,v_static(:,c(2)), 'Color', colortwo, 'LineWidth', 1)
    xlabel('$t$ [h]')
    ylabel('$v$ [p.u.]')
    ylim([0.98 1.1])
    xlim([0 12])
    box on
    
figure(3)
    hold on
    line([0 t(end)/60/60], [params.QMIN(1) params.QMIN(1)], 'Color', colorone, 'LineStyle', '--')
    line([0 t(end)/60/60], [params.QMIN(2) params.QMIN(2)], 'Color', colortwo, 'LineStyle', '--')
    line([0 t(end)/60/60], [0 0], 'Color', 'black', 'LineStyle', '-')
    plot(t/60/60,q_static(:,1), 'Color', colorone, 'LineWidth', 1)
    plot(t/60/60,q_static(:,2), 'Color', colortwo, 'LineWidth', 1)
    xlabel('$t$ [h]')
    ylabel('$q_G$ [MVAR]')
    ylim([min(params.QMIN)-0.1 0.1])
    xlim([0 12])
    box on
    
figure(4)
    hold on
    line([0 t(end)/60/60], [params.VMAX params.VMAX], 'Color', 'black', 'LineStyle', '--')
    line([0 t(end)/60/60], [1 1], 'Color', 'black', 'LineStyle', '-')
    plot(t/60/60,v_incremental, 'Color', colorgrey, 'LineWidth', 0.5)
    plot(t/60/60,v_incremental(:,c(1)), 'Color', colorone, 'LineWidth', 1)
    plot(t/60/60,v_incremental(:,c(2)), 'Color', colortwo, 'LineWidth', 1)
    xlabel('$t$ [h]')
    ylabel('$v$ [p.u.]')
    ylim([0.98 1.1])
    xlim([0 12])
    box on
    
figure(5)
    hold on
    line([0 t(end)/60/60], [params.QMIN(1) params.QMIN(1)], 'Color', colorone, 'LineStyle', '--')
    line([0 t(end)/60/60], [params.QMIN(2) params.QMIN(2)], 'Color', colortwo, 'LineStyle', '--')
    line([0 t(end)/60/60], [0 0], 'Color', 'black', 'LineStyle', '-')
    plot(t/60/60,q_incremental(:,1), 'Color', colorone, 'LineWidth', 1)
    plot(t/60/60,q_incremental(:,2), 'Color', colortwo, 'LineWidth', 1)
    xlabel('$t$ [h]')
    ylabel('$q_G$ [MVAR]')
    ylim([min(params.QMIN)-0.1 0.1])
    xlim([0 12])
    box on
    
figure(6)
    hold on
    line([0 t(end)/60/60], [params.VMAX params.VMAX], 'Color', 'black', 'LineStyle', '--')
    line([0 t(end)/60/60], [1 1], 'Color', 'black', 'LineStyle', '-')
    plot(t/60/60,v_networked, 'Color', colorgrey, 'LineWidth', 0.5)
    plot(t/60/60,v_networked(:,c(1)), 'Color', colorone, 'LineWidth', 1)
    plot(t/60/60,v_networked(:,c(2)), 'Color', colortwo, 'LineWidth', 1)
    xlabel('$t$ [h]')
    ylabel('$v$ [p.u.]')
    ylim([0.98 1.1])
    xlim([0 12])
    box on
    
figure(7)
    hold on
    line([0 t(end)/60/60], [params.QMIN(1) params.QMIN(1)], 'Color', colorone, 'LineStyle', '--')
    line([0 t(end)/60/60], [params.QMIN(2) params.QMIN(2)], 'Color', colortwo, 'LineStyle', '--')
    line([0 t(end)/60/60], [0 0], 'Color', 'black', 'LineStyle', '-')
    plot(t/60/60,q_networked(:,1), 'Color', colorone, 'LineWidth', 1)
    plot(t/60/60,q_networked(:,2), 'Color', colortwo, 'LineWidth', 1)
    xlabel('$t$ [h]')
    ylabel('$q_G$ [MVAR]')
    ylim([min(params.QMIN)-0.1 0.1])
    xlim([0 12])
    box on

figure(8)
    hold on
    line([0 t(end)/60/60], [0 0], 'Color', 'black', 'LineStyle', '-')
    plot(t/60/60,pvproduction(:,1), 'Color', colorone, 'LineWidth', 1)
    plot(t/60/60,pvproduction(:,2), 'Color', colortwo, 'LineWidth', 1)
    xlabel('$t$ [h]')
    ylabel('$p_G$ [MW]')
    xlim([0 12])
    box on

figure(9)
    hold on
    line([0 t(end)/60/60], [0 0], 'Color', 'black', 'LineStyle', '-')
    plot(t/60/60,p, 'Color', colorgrey)
    plot(t/60/60,p(:,15), 'Color', 'black', 'LineWidth', 1)
    xlabel('$t$ [h]')
    ylabel('$-p_L$ [MW]')
    xlim([0 12])
    box on


