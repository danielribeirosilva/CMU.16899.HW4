function traj = lqr_run_controller_in_linear_sim(lqr_traj, simulate_f, params, model_bias, open_loop_flag)

     % Authors: Pieter Abbeel (pabbeel@cs.berkeley.edu)
     %          Adam Coates (acoates@cs.stanford.edu)

% run lqr controllers on non-linear dynamics
traj.target_x = lqr_traj.target_x;
traj.t = lqr_traj.t;

length = size(lqr_traj.t,1);
x = lqr_traj.nom_x(1,:);
for i=1:length-1
  
  dx = compute_dx(lqr_traj.target_x(i,:), x);
  %% add observation noise here ...
  if(open_loop_flag==1)
      u = (lqr_traj.K{i}(:,end))';
  else
      u = (lqr_traj.K{i} *dx')';
  end

  
  traj.dx(i,:) = dx;
  traj.x(i,:) = x;
  traj.u(i,:) = u;
  
  sim_time = lqr_traj.t(i+1) - lqr_traj.t(i);
  x = compose_dx( (lqr_traj.A{i} * dx' + lqr_traj.B{i} * u')',  lqr_traj.target_x(i+1,:) );
%  x = feval(simulate_f, x, u, sim_time, params,model_bias(i,:));%state-noise: + randn(1,2);
end

dx = compute_dx(lqr_traj.target_x(length,:),x);
traj.dx(length,:) = dx;
traj.x(length,:) = x;
traj.u(length,:) = u;

