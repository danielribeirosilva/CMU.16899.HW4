function [qscore, rscore] = score_lqr_trajectory_generic_reward(traj, reward)

     % Authors: Pieter Abbeel (pabbeel@cs.berkeley.edu)
     %          Adam Coates (acoates@cs.stanford.edu)

qscore = 0;
rscore = 0;

Q = diag(reward.state_multipliers);
R = diag(reward.input_multipliers);

for i=1:size(traj.x,1)
  dx = traj.dx(i,:);
  qscore = qscore + dx * Q * dx';
  u = traj.u(i,:);
  rscore = rscore + u * R * u';
end

