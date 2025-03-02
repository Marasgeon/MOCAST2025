%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ANFIS-PID Controller for Two-Link Cartesian Robot with Surface Contact %
% This script implements an ANFIS-PID controller for a two-link Cartesian%
% robot with its end-effector in contact with an inclined plane          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc; format compact;

% System Parameters
m1 = 0.5;     
m2 = 1;       
g = 9.81;     
phi = pi/4;   
ls = 0.2;     
mu = 0.1;     
lambda0 = 1;  

tanPhi = tan(phi);
sinPhi = sin(phi);

% Simulation Parameters
Ts = 0.001;   
Tf = 15;       
t = 0:Ts:Tf;  
N = length(t);

% Reference Trajectory
q1_init = 0.2;   
q1_final = 0.5;  

tt = t/Tf;
q1_ref = q1_init + (q1_final - q1_init) * (6*tt.^5 - 15*tt.^4 + 10*tt.^3);
dq1_ref = (q1_final - q1_init) * (30*tt.^4 - 60*tt.^3 + 30*tt.^2) / Tf;
ddq1_ref = (q1_final - q1_init) * (120*tt.^3 - 180*tt.^2 + 60*tt) / Tf^2;

q2_ref = q1_ref * tanPhi;
dq2_ref = dq1_ref * tanPhi;
ddq2_ref = ddq1_ref * tanPhi;

% ANFIS-PID Parameters
error_range = [-0.1 0.1];   
derror_range = [-0.2 0.2];  

anfis_Kp = sugfis;
anfis_Ki = sugfis;
anfis_Kd = sugfis;

anfis_Kp = addInput(anfis_Kp, error_range, 'Name', 'error');
anfis_Kp = addInput(anfis_Kp, derror_range, 'Name', 'derror');
anfis_Ki = addInput(anfis_Ki, error_range, 'Name', 'error');
anfis_Ki = addInput(anfis_Ki, derror_range, 'Name', 'derror');
anfis_Kd = addInput(anfis_Kd, error_range, 'Name', 'error');
anfis_Kd = addInput(anfis_Kd, derror_range, 'Name', 'derror');

anfis_Kp = addMF(anfis_Kp, 'error', 'trimf', [-0.2 -0.1 0], 'Name', 'negative');
anfis_Kp = addMF(anfis_Kp, 'error', 'trimf', [-0.1 0 0.1], 'Name', 'zero');
anfis_Kp = addMF(anfis_Kp, 'error', 'trimf', [0 0.1 0.2], 'Name', 'positive');
anfis_Kp = addMF(anfis_Kp, 'derror', 'trimf', [-0.4 -0.2 0], 'Name', 'negative');
anfis_Kp = addMF(anfis_Kp, 'derror', 'trimf', [-0.2 0 0.2], 'Name', 'zero');
anfis_Kp = addMF(anfis_Kp, 'derror', 'trimf', [0 0.2 0.4], 'Name', 'positive');

anfis_Ki = addMF(anfis_Ki, 'error', 'trimf', [-0.2 -0.1 0], 'Name', 'negative');
anfis_Ki = addMF(anfis_Ki, 'error', 'trimf', [-0.1 0 0.1], 'Name', 'zero');
anfis_Ki = addMF(anfis_Ki, 'error', 'trimf', [0 0.1 0.2], 'Name', 'positive');
anfis_Ki = addMF(anfis_Ki, 'derror', 'trimf', [-0.4 -0.2 0], 'Name', 'negative');
anfis_Ki = addMF(anfis_Ki, 'derror', 'trimf', [-0.2 0 0.2], 'Name', 'zero');
anfis_Ki = addMF(anfis_Ki, 'derror', 'trimf', [0 0.2 0.4], 'Name', 'positive');

anfis_Kd = addMF(anfis_Kd, 'error', 'trimf', [-0.2 -0.1 0], 'Name', 'negative');
anfis_Kd = addMF(anfis_Kd, 'error', 'trimf', [-0.1 0 0.1], 'Name', 'zero');
anfis_Kd = addMF(anfis_Kd, 'error', 'trimf', [0 0.1 0.2], 'Name', 'positive');
anfis_Kd = addMF(anfis_Kd, 'derror', 'trimf', [-0.4 -0.2 0], 'Name', 'negative');
anfis_Kd = addMF(anfis_Kd, 'derror', 'trimf', [-0.2 0 0.2], 'Name', 'zero');
anfis_Kd = addMF(anfis_Kd, 'derror', 'trimf', [0 0.2 0.4], 'Name', 'positive');

anfis_Kp = addOutput(anfis_Kp, [0 100], 'Name', 'Kp');
anfis_Ki = addOutput(anfis_Ki, [0 50], 'Name', 'Ki');
anfis_Kd = addOutput(anfis_Kd, [0 20], 'Name', 'Kd');

for i = 1:9  
    anfis_Kp = addMF(anfis_Kp, 'Kp', 'constant', 50, 'Name', ['out' num2str(i)]);
    anfis_Ki = addMF(anfis_Ki, 'Ki', 'constant', 20, 'Name', ['out' num2str(i)]);
    anfis_Kd = addMF(anfis_Kd, 'Kd', 'constant', 10, 'Name', ['out' num2str(i)]);
end

% Fuzzy Rules Base
ruleList_Kp = [
    1 1 1 1 1
    1 2 2 1 1
    1 3 3 1 1
    2 1 4 1 1
    2 2 5 1 1
    2 3 6 1 1
    3 1 7 1 1
    3 2 8 1 1
    3 3 9 1 1
];
anfis_Kp = addRule(anfis_Kp, ruleList_Kp);
anfis_Ki = addRule(anfis_Ki, ruleList_Kp);
anfis_Kd = addRule(anfis_Kd, ruleList_Kp);


anfis_Kp.output.mf(1).params = 80;  
anfis_Kp.output.mf(2).params = 70;  
anfis_Kp.output.mf(3).params = 60;  
anfis_Kp.output.mf(4).params = 70;  
anfis_Kp.output.mf(5).params = 50;  
anfis_Kp.output.mf(6).params = 70;  
anfis_Kp.output.mf(7).params = 60;  
anfis_Kp.output.mf(8).params = 70;  
anfis_Kp.output.mf(9).params = 80;  

anfis_Ki.output.mf(1).params = 5;   
anfis_Ki.output.mf(2).params = 15;  
anfis_Ki.output.mf(3).params = 5;   
anfis_Ki.output.mf(4).params = 15;  
anfis_Ki.output.mf(5).params = 25;  
anfis_Ki.output.mf(6).params = 15;  
anfis_Ki.output.mf(7).params = 5;   
anfis_Ki.output.mf(8).params = 15;  
anfis_Ki.output.mf(9).params = 5;   

anfis_Kd.output.mf(1).params = 15;  
anfis_Kd.output.mf(2).params = 10;  
anfis_Kd.output.mf(3).params = 5;   
anfis_Kd.output.mf(4).params = 10;  
anfis_Kd.output.mf(5).params = 5;   
anfis_Kd.output.mf(6).params = 10;  
anfis_Kd.output.mf(7).params = 5;   
anfis_Kd.output.mf(8).params = 10;  
anfis_Kd.output.mf(9).params = 15;  

% Variables Initialization
x = zeros(5, N);
x(:, 1) = [q1_init; q1_init*tanPhi; 0; 0; lambda0];

u = zeros(2, N);  

err_q1 = zeros(1, N);
err_q1_sum = 0;
err_q1_prev = 0;

Kp_history = zeros(1, N);
Ki_history = zeros(1, N);
Kd_history = zeros(1, N);

% Simulation
for k = 1:N-1
    
    q1 = x(1, k);
    q2 = x(2, k);
    dq1 = x(3, k);
    dq2 = x(4, k);
    lambda = x(5, k);
    
    err_q1(k) = q1_ref(k) - q1;
     
    if k > 1
        derr_q1 = (err_q1(k) - err_q1(k-1)) / Ts;
    else
        derr_q1 = 0;
    end
    
    Kp = evalfis(anfis_Kp, [err_q1(k), derr_q1]);
    Ki = evalfis(anfis_Ki, [err_q1(k), derr_q1]);
    Kd = evalfis(anfis_Kd, [err_q1(k), derr_q1]);
    
    Kp_history(k) = Kp;
    Ki_history(k) = Ki;
    Kd_history(k) = Kd;
    
    err_q1_sum = err_q1_sum + err_q1(k) * Ts;
    
    if k > 1
        derr_q1_dt = (err_q1(k) - err_q1_prev) / Ts;
    else
        derr_q1_dt = 0;
    end
    err_q1_prev = err_q1(k);
    
    
    ff_term = (m1 + m2) * ddq1_ref(k) + (m1 + m2) * g * sinPhi;
    
    pid_term = Kp * err_q1(k) + Ki * err_q1_sum + Kd * derr_q1_dt;
    
    u(1, k) = ff_term + pid_term;
    
    fx = mu * lambda;
    fz = mu * lambda * tanPhi;
    
    u(2, k) = -tanPhi * m2 * ddq1_ref(k) + m2 * g - lambda0 * tanPhi - fz;
     
    M = [m1 + m2, 0; 0, m2];
      
    G = [(m1 + m2) * g * sinPhi; m2 * g];
        
    J = [1, 0; 0, 1]; 
    
    D = [-tanPhi; 1];
     
    F = [fx; fz];
     
    tau = [u(1, k); u(2, k)];
    
    ddq1 = (tau(1) - G(1) - fx - lambda * (-tanPhi)) / (m1 + m2);
    
    ddq2 = tanPhi * ddq1;
    
    dlambda = 0;  
    
    x(1, k+1) = q1 + dq1 * Ts;
    x(2, k+1) = q2 + dq2 * Ts;
    x(3, k+1) = dq1 + ddq1 * Ts;
    x(4, k+1) = dq2 + ddq2 * Ts;
    x(5, k+1) = lambda + dlambda * Ts;
end

% Plot Results
figure(1);
subplot(2,1,1);
plot(t, q1_ref, 'g--', 'LineWidth', 2); hold on;
plot(t, x(1,:), 'm-', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Position [m]');
title('Joint 1 Position Tracking');
legend('Reference', 'Actual');

subplot(2,1,2);
plot(t, q2_ref, 'g--', 'LineWidth', 2); hold on;
plot(t, x(2,:), 'm-', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Position [m]');
title('Joint 2 Position Tracking');
f1 = gcf;
exportgraphics(f1,'figure1.jpg','Resolution',600)

figure(2);
subplot(2,1,1);
tracking_error_q1 = q1_ref - x(1,:);
plot(t, tracking_error_q1 * 1000, 'm-', 'LineWidth', 1.5);  
grid on;
xlabel('Time [s]');
ylabel('Error [mm]');
title('Joint 1 Tracking Error');

subplot(2,1,2);
tracking_error_q2 = q2_ref - x(2,:);
plot(t, tracking_error_q2 * 1000, 'm-', 'LineWidth', 1.5);  
grid on;
xlabel('Time [s]');
ylabel('Error [mm]');
title('Joint 2 Tracking Error');
f2 = gcf;
exportgraphics(f2,'figure2.jpg','Resolution',600)

figure(3);
subplot(2,1,1);
plot(t, u(1,:), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Torque [Nm]');
title('Joint 1 Control Input');

subplot(2,1,2);
plot(t, u(2,:), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Torque [Nm]');
title('Joint 2 Control Input');
f3 = gcf;
exportgraphics(f3,'figure3.jpg','Resolution',600)

figure(4);
subplot(3,1,1);
plot(t(1:end-1), Kp_history(1:end-1), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Kp');
title('Proportional Gain Adaptation');

subplot(3,1,2);
plot(t(1:end-1), Ki_history(1:end-1), 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Ki');
title('Integral Gain Adaptation');

subplot(3,1,3);
plot(t(1:end-1), Kd_history(1:end-1), 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Kd');
title('Derivative Gain Adaptation');
f4 = gcf;
exportgraphics(f4,'figure4.jpg','Resolution',600)

figure(5);
constraint_violation = abs(x(2,:) - x(1,:) * tanPhi);
plot(t, constraint_violation * 1000, 'b-', 'LineWidth', 1.5);  
grid on;
xlabel('Time [s]');
ylabel('Violation [mm]');
title('Constraint Violation');
f5 = gcf;
exportgraphics(f5,'figure5.jpg','Resolution',600)

figure(6);
plot(x(1,:), x(3,:), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Position q1 [m]');
ylabel('Velocity dq1 [m/s]');
title('Phase Plane Plot for Joint 1');
f6 = gcf;
exportgraphics(f6,'figure6.jpg','Resolution',600)

figure(7);
plot(t, x(5,:), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('λ [N]');
title('Contact Force (λ)');
f7 = gcf;
exportgraphics(f7,'figure7.jpg','Resolution',600)

ss_error_q1 = abs(q1_ref(end) - x(1,end)) * 1000;  
ss_error_q2 = abs(q2_ref(end) - x(2,end)) * 1000;  

fprintf('Steady-state error for q1: %.3f mm\n', ss_error_q1);
fprintf('Steady-state error for q2: %.3f mm\n', ss_error_q2);
fprintf('Maximum control input for joint 1: %.3f N\n', max(abs(u(1,:))));
fprintf('Maximum control input for joint 2: %.3f N\n', max(abs(u(2,:))));
fprintf('Settling time (±0.5mm): %.3f s\n', findSettlingTime(t, tracking_error_q1*1000, 0.5));

% t_settling function declaration
function t_settling = findSettlingTime(t, error, threshold)
    
    n = length(error);
    for i = 1:n
        if all(abs(error(i:end)) <= threshold)
            t_settling = t(i);
            return;
        end
    end
    t_settling = inf;  
end