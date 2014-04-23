function [lqr_traj, result_traj, im_trajs] = ...
	ddp_for_trajectory_following(start_state, reward, target_traj, simulate_f, model, idx, model_bias_est, DT)

     % Authors: Pieter Abbeel (pabbeel@cs.berkeley.edu)
     %          Adam Coates (acoates@cs.stanford.edu)

open_loop_flag = 0;
magic_factors = [.95 .8 .5 .2 .1 .05 .02 .01 .005 .002 .001 0];
num_ddp_iters = length(magic_factors);
H = size(target_traj.x,1);
nom_traj = target_traj;
for ddp_iter = 1:num_ddp_iters
	ddp_iter
	magic_factor = magic_factors(ddp_iter);
	lqr_traj = lqr_backups_for_trajectory_following(nom_traj, target_traj, simulate_f, model, idx, model_bias_est, reward, magic_factor);
	nom_traj = lqr_run_controller_in_nonlinear_sim(start_state, lqr_traj, simulate_f, model, idx, model_bias_est, magic_factor, open_loop_flag);
	[ qscore, rscore ] = score_lqr_trajectory(nom_traj, reward);
	qscore + rscore ;  % should decrease
	im_trajs{ddp_iter} = nom_traj;
%  	figure; plot(nom_traj.x(:,idx.q));
% %  	figure; plot(nom_traj.x(:,idx.pqr(2)));
%  	figure; plot(nom_traj.x(:,idx.u_prev));
%  	figure; plot(nom_traj.x(:,idx.ned));
end


result_traj = lqr_run_controller_in_nonlinear_sim(start_state,lqr_traj, simulate_f, model, idx, model_bias_est, magic_factor, open_loop_flag);
%figure; plot(result_traj.u);
%result_traj_open_loop = lqr_nonlinear_trajectory(lqr_traj, simulate_f, params, model_bias_est, magic_factor, 1);
% figure; plot(result_traj.x(:,idx.ned));
