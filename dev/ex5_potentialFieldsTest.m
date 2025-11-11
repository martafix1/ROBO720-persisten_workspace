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


%% repulsiveField

% get the eq for coefs
syms a b x0 x1 x10 y 

eqn_0 = 0 == b*(1/x0 - 1/a)
eqn_1 = 1 == b*(1/x1 - 1/a)
eqn_10 = 10 == b*(1/x10 - 1/a)
eqn2_0 = 0 == b*(1/x0 - 1/a)^2
eqn2_1 = 1 == b*(1/x1 - 1/a)^2
eqn2_10 = 10 == b*(1/x10 - 1/a)^2

eqn3_0 = 0 == b*(1/x0 - 1/a)* 1/(x0.^2)
eqn3_1 = 1 == b*(1/x1 - 1/a)* 1/(x1.^2)
eqn3_10 = 10 == b*(1/x10 - 1/a)* 1/(x10^2)

sol1 = solve([eqn_0 eqn_1], [a b])

sol1_just10 = solve([eqn_10], [x10])

sol2 = solve([eqn2_0 eqn2_1], [a b])

sol2_just10 = solve([eqn2_10], [x10])

simplify(sol2.a) 
simplify(sol2.b) 

sol3 = solve([eqn3_0 eqn3_1], [a b]) 
sol3_just10 = solve([eqn3_10], [x10]) 



x0 = 2
x1 = 1
% x10 = 0.2


% sol1
% a1= -(9*x1*x10)/(x1 - 10*x10)
% b1= (9*x1*x10)/(x1 - x10)

% sol1_0
a1= x0
b1= (x0*x1)/(x0 - x1)
x1_10 = 1/(1/a1 + 10/b1)

% sol2
% a2= (x1*x10*(x1 - 10*x10 - 10^(1/2)*x1 + 10^(1/2)*x10))/(x1^2 - 10*x10^2)
% b2= -(x1^2*x10^2*(2*10^(1/2) - 11))/(x1 - x10)^2

% sol2_0
a2= x0
b2= (x0^2*x1^2)/(x0^2 - 2*x0*x1 + x1^2)
x2_10 = 1/(1/a2 + 10^(1/2)*(1/b2)^(1/2))

% sol3_0

a3 = x0
b3 = (x0*x1^3)/(x0 - x1)



d_ = 0:0.01:2.5;
U = b1*(1./d_-1/a1);
U(d_ > x0) = 0;
U2 = b2*(1./d_-1/a2).^2;
U2(d_ > x0) = 0;
U3 = b3*(1./d_ - 1/a3)* 1./(d_.^2);
U3(d_ > x0) = 0;



plot(d_,U)
ylim([0 15])

hold on

plot(d_,U2)

plot(d_,U3)

hold off
legend(["U1", "U2","U3"])
grid("on")
