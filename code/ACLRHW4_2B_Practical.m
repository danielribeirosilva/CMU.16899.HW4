%% Inverse Optimal Control

% clean up the matlab environment
clear; clc; close all;

% run initialization of some paths and variables
init_setup;
load('K_ss.mat');

target_traj.x = repmat(target_hover_state',H,1);
target_traj.u = zeros(H,4); %% recall: u is change in controls, which is zero for stationary hover
target_traj.t = ((0:length(target_traj.x)-1)*dt)';

%% (a) derive a cost function for which the given controller is optimal
simulate_f = @f_heli;
open_loop_flag = 0;
magic_factor = 0;
nom_traj = target_traj;
model_bias_est = zeros(H,6);
gamma = 1e-2; % step size

% initialize Q and R
Q = eye(21);
R = eye(4);

% Compute features: summed x*x' for demonstrated trajectory
x = start_state; 
demo_sum = x*x';
for j=1:H-1
  dx = compute_dx(target_hover_state, x);
  u = K_ss*dx;
  demo_traj.x(j,:) = x';
  x = A*x + B*u;
  demo_sum = demo_sum + x*x';
end

% sub-gradient optimization
for i=1:30
    fprintf('Iteration %d\n',i);
    % run LQR, get linear controller
    lqr_traj = LQR_with_full_Q(nom_traj, target_traj, simulate_f, model, idx, model_bias_est, Q, R, magic_factor);
    % Forward sum, find LQR trajectory
    x = start_state;
    LQR_sum = x*x';
    for j=1:H-1
        dx = compute_dx(target_hover_state, x);
        u = lqr_traj.K{j}*dx;
        nom_traj.x(j,:) = x';
        x = lqr_traj.A{j}*x + lqr_traj.B{j}*u;
        if ~isreal(x*x'), break; end
        LQR_sum = LQR_sum + x*x'; % Compute features: summed x*x' for LQR trajectory
    end
    % calculate sub-gradient of Q and update
    subgradientQ = demo_sum - LQR_sum;
    Q = Q - gamma*subgradientQ;
end

% now make Q positive-definite
[V,D]=eig(Q);
d=diag(D);
d(d<=0)=eps;
Q=V*diag(d)*V';


%% (b) derive a controller for that cost function again using LQR
nom_traj = target_traj;
lqr_traj = LQR_with_full_Q(nom_traj, target_traj, simulate_f, model, idx, model_bias_est, Q, R, magic_factor);

%% plot results

x = start_state; 
for i=1:H-1
  traj.x(i,:) = x';
  dx = compute_dx(target_hover_state, x);
  u = lqr_traj.K{i}*dx;
  x = lqr_traj.A{j}*x + lqr_traj.B{j}*u;
end

figure; plot(traj.x(:,idx.ned)); legend('north', 'east', 'down');
hold on; plot(target_traj.x(:,idx.ned),'.');ylim([-30,30]);

figure; plot(traj.x(:,idx.q)); legend('qx', 'qy', 'qz', 'qw');
hold on; plot(target_traj.x(:,idx.q),'.');ylim([-30,30]);

figure; plot(traj.x(:,idx.u_prev)); legend('aileron','elevator','rudder','collective');
hold on; plot(target_traj.x(:,idx.u_prev),'.');ylim([-10,10]);