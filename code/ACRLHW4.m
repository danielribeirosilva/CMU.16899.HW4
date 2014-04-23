% clean up the matlab environment
clear; clc; close all;

% run initialization of some paths and variables
init_setup;

H = 600;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% we can start working now %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% let's use DDP to do some aerobatics: we 'll fly a nose-in funnel                                        %% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Q2: Use DDP to find a controller to fly the trajectory specified below.
%% Hand in: (i) plots of the target trajectory's and the resulting trajectory's ned, q and u_prev, 
%%          (ii) Comment briefly on how well they match---if in some axes they don't
%%               match, explain why this might be the case.
%%          (iii) Explain how the DDP code differs from generic DDP code. Explain as well what the"magic factor" is accomplishing.
%%                Is it necessary? How else might this be accomplished. 

%% fill in target_traj.x, target_traj.u, target_traj.t
%%
%% (1) we hover for 5 seconds (100 tics) at (-5,0,0), facing North (q=[0;0;0;1])
%% (2) we fly 4 times in a row a 5m radius circle at a speed of 2*pi*5 / 5 -- i.e., we
%% complete each circle in 5 seconds; throughout the circle, the nose of
%% the helicopter always points to (0,0,0)  [400 tics]
%% (3) we hover for 5 seconds at (-5,0,0), facing North [100 tics]

%% [--> total number of tics is 600]

%% note: let a quaternion q = [axis * sin(theta/2); cos(theta/2)]
%%      --> replacing theta by theta + 2*pi gives qbar = -q; 
%%          qbar does represent the same orientation though!  -- 
%%          after 1 funnel --> target q will not be back to [0;0;0;1], but it will
%%          be [0; 0; 0; -1]


hover_period = 5;
hover_numtics = 5/dt;
num_funnels = 4;
f_radius = 5;
f_period = 5;
f_numtics = 5/dt;
f_start = hover_numtics+1;

hover_quaternion_target = [0;0;0;1]; 
hover_ned_target = [-f_radius; 0; 0];
target_hover_state = [ zeros(4,1); zeros(4,1); zeros(3,1); zeros(3,1); zeros(3,1); hover_quaternion_target];
target_hover_state(idx.ned) = hover_ned_target;


target_traj.x = repmat(target_hover_state',H,1);
target_traj.u = zeros(H,4); %% recall: u is change in controls, which is zero for stationary hover
target_traj.t = ((0:length(target_traj.x)-1)*dt)';

% now splice in the funnel maneuver at the appropriate times:
% we set up the state sequence for two funnels (and then repeat that twice
% into the target_traj)   --- We can't use four times 1 funnel, due to the
% quaternion wrap-around property (explained above).

two_funnel_phase = ((0:(4*pi)/(2*f_numtics-1):4*pi))';
two_funnel_ned(:,1) = -f_radius*cos(two_funnel_phase);
two_funnel_ned(:,2) = -f_radius*sin(two_funnel_phase);
two_funnel_ned(:,3) = 0*two_funnel_phase;
two_funnel_q = zeros(200,4);
for i=1:200, two_funnel_q(i,:) = euler_to_q([0,0,two_funnel_phase(i)])'; end
two_funnel_ned_dot = [f_radius*sin(two_funnel_phase), -f_radius*cos(two_funnel_phase), 0*two_funnel_phase];
two_funnel_pqr = [zeros(200,2), 2*pi/100*ones(200,1)];

for k=1:num_funnels/2
	target_traj.x(f_start+(k-1)*2*f_numtics:f_start+k*2*f_numtics-1, idx.ned_dot) = two_funnel_ned_dot;
	target_traj.x(f_start+(k-1)*2*f_numtics:f_start+k*2*f_numtics-1, idx.ned) = two_funnel_ned;
	target_traj.x(f_start+(k-1)*2*f_numtics:f_start+k*2*f_numtics-1, idx.pqr) = two_funnel_pqr;
	target_traj.x(f_start+(k-1)*2*f_numtics:f_start+k*2*f_numtics-1, idx.q) = two_funnel_q;
end



% DDP here

%% Here is some suggested code for getting you started with using DDP:

start_state = target_hover_state;
model_bias_est = zeros(H,6);

u_prev_mult = 0; u_delta_prev_mult = 1000;
ned_dot_mult = 0.01; ned_mult = 10;
pqr_mult = 1; q_mult = 10;
always1state_mult = 0;

reward.state_multipliers = [ u_prev_mult * ones(1,4)   u_delta_prev_mult * ones(1,4)  ned_dot_mult * ones(1,3)  ned_mult*ones(1,3)  ...
	pqr_mult * ones(1,3)  q_mult * ones(1,3)  always1state_mult * ones(1,1)]';
reward.input_multipliers = ones(4,1)*0;

simulate_f = @f_heli;

[lqr_traj, result_traj, im_trajs] = ddp_for_trajectory_following(start_state, reward, target_traj, simulate_f, model, idx, model_bias_est, dt);


% let's check how well we perform:
% note: for any given system, we could run all the code we have in matlab,
% and then just use the lines below "on-board" or in a C-code control loop

x = target_hover_state; 
for i=1:H-1
  dx = compute_dx(lqr_traj.target_x(i,:)', x);
  u = (lqr_traj.K{i}*dx);
  traj.x(i,:) = x';
  x = simulate_f(x, u, dt, model, idx);
end
traj.x(H,:) = x';


figure; plot(traj.x(:,idx.ned)); legend('north', 'east', 'down');
hold on; plot(target_traj.x(:,idx.ned),'.');

figure; plot(traj.x(:,idx.q)); legend('qx', 'qy', 'qz', 'qw');
hold on; plot(target_traj.x(:,idx.q),'.');

figure; plot(traj.x(:,idx.u_prev)); legend('aileron','elevator','rudder','collective');
hold on; plot(target_traj.x(:,idx.u_prev),'.');


