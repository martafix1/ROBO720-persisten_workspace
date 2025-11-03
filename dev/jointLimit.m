function y = jointLimit(q_, qdot_, )
global GV

safetyLimit_deg = 45; % degrees
safetyLimit_rad = safetyLimit_deg * (3.14 / 180.0); % convert to radians

decayRate = 0.5; % decay rate
velCommandGain = 1; % velocity command gain


q_z  = q_  + (qdot_ / GV.freq);

dist_max = abs(GV.pos_lim_max  - q_z );
dist_min = abs(GV.pos_lim_min  - q_z );

desired_pos_step = 0; % Initialize desired_pos_step
if (dist_max < safetyLimit_rad) && (qdot_  > 0)
desired_pos_step = (q_  - position_lim_MAX ) * decayRate;
elseif (dist_min < safetyLimit_rad) && (qdot_  < 0)
desired_pos_step = (q_  - position_lim_MIN ) * decayRate;
end

qd_dot_add  = desired_pos_step * freq * velCommandGain;


y = [qd_dot_add ,q_z,desired_pos_step];

