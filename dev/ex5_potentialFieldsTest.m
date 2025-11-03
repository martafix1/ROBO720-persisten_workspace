%  test of ex5 potential fields


vel_lim_max = 2.1750;
vel_lim_min = -vel_lim_max;

pos_lim_max = 2.8973;
pos_lim_min = -2.8973;

GV.pos_lim_max = pos_lim_max;
GV.pos_lim_min = pos_lim_min;

% Create a bus from this structure
Simulink.Bus.createObject(GV);

w = 20
damp = 1
Kd = damp*w*2
Kp = w*w

GV.freq = 1000



%% joint fields




% q_ = 2.8
% qdot_ = 2.1
% 
% 
% viscoseConstant = 10
% 
% 
% safetyLimit_deg = 10 % degrees
% safetyLimit_rad = safetyLimit_deg * (3.14 / 180.0)% convert to radians
% 
% predictionCycles = 20;
% 
% viscos_array = []
% repuls_array = []
% 
% q_ = 2:0.01:3 ;
% 
% 
% for i = 1:size(q_,2)
% distLim = (pos_lim_max-safetyLimit_rad)-q_
% 
% if(distLim>0)
% viscosity = distLim*viscoseConstant
% else
% viscosity = 0;
% end
% 
% if(qdot_ > 0)
%     repulse = qdot_ *  viscosity;
% end
% 
% viscos_array(i) = viscosity;
% repuls_array(i) = repulse;
% 
% end 


