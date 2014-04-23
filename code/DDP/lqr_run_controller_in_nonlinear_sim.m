function traj = lqr_run_controller_in_nonlinear_sim(start_state, lqr_traj, simulate_f, model, idx, model_bias, magic_factor, open_loop_flag)

     % Authors: Pieter Abbeel (pabbeel@cs.berkeley.edu)
     %          Adam Coates (acoates@cs.stanford.edu)

% run lqr controllers on non-linear dynamics
if(open_loop_flag == 1)
	'running in open loop ...'
end

traj.target_x = lqr_traj.target_x;
traj.t = lqr_traj.t;
traj.u = zeros(size(lqr_traj.nom_u));

H = length(lqr_traj.t);
x = start_state; %lqr_traj.nom_x(1,:)';
for i=1:H-1
  
  dx = compute_dx(lqr_traj.target_x(i,:)', x);
  %% add observation noise here ...
  if(open_loop_flag==1)
      u = (lqr_traj.K{i}(:,end));
  else
      u = (lqr_traj.K{i}*dx);
  end

  
  traj.dx(i,:) = dx';
  traj.x(i,:) = x';
  traj.u(i,:) = u';
  
  sim_time = lqr_traj.t(i+1) - lqr_traj.t(i);
  x = feval(simulate_f, x, u, sim_time, model, idx, model_bias(i,:)', magic_factor, traj.target_x(i+1,:)');%state-noise: + randn(1,2);
  %x(1:end-4) = x(1:end-4)+randn(length(x)-4,1)*.01;
end

dx = compute_dx(lqr_traj.target_x(H,:)',x);
traj.dx(H,:) = dx';
traj.x(H,:) = x';
traj.u(H,:) = traj.u(H-1,:);

