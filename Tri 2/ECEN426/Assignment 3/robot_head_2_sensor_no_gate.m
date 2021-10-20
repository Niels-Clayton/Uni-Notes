%===============================================================================
% ECEN426 Advanced Mechatronic Systems
% Assignment One 2021
%
% This file is skeleton code that implements a Kalman filter to estimate the
% pose of a robot head/neck in the roll dimension.
%
% C.Hollitt
% 20th September 2021
%===============================================================================
%-------------------------------------------------------------------------------
% Form the model.

dt = 10e-3; % 10 ms time interval.
T_max = 20; % Duration of the simulation.
t=[0:1:T_max/dt]'.*dt;

% State vector
% [theta, omega, alpha]

A= [1 dt 0.5*dt^2; 
    0 1 dt; 
    0 0 0];

B= [0 0 1]';

C_1= [0 0 1]; 

C_2= [1 0 0];

D= 0;

% Noise model -------------------

G = [0.5*dt^2 ; dt ; 1];
var_acceleration = 10; % Variance of the acceleration noise on the head.

Q = G * var_acceleration * (G');

R_1 = 1;   % Variance in the acceleration measurement (sensor 1).
R_2 = 0.5; % Variance in the position measurement (sensor 2).

%-------------------------------------------------------------------------------
% Initial  Conditions.

% Actual position that the neck begins the simulation.
x0 = [-2, 0, 10]'; % Don't change this - it is the true initial state that you 
                   % "don't know".

x(:,1) = x0; % Set the initial state for the simulation.

% %-----------------------------------------------------------------------------
% % Run simulation 

I=eye(3);
L_store = [[P_post * C_1' * inv( C_1 * P_post *C_1' + R_1)]'];

% Set the threshold value. If the new measurment is threshold times larger
% than the previous, it will be replaces by the previous
threshold = 50;

error_ks = [30, 60, 120, 150, 300, 600, 660,730, 900, 930];

for k=2:length(t);
  % x(k) = Ax(k-1) + Bu(k) + w(k)
  w = G*sqrt(var_acceleration)*randn(1,1);

  x(:,k) = A*x(:,k-1) + w;   % u=0, so omit the B*u(k) term.
  
  x_prior(:,k) = A*x_post(:,k-1);
  P_prior = A*P_post*A' + Q;
  
  % Update the first sensor
  S1_update_interval = 1; 
  if mod(k, S1_update_interval)==0 
    v = sqrt(R_1)*randn(1,1); % Find noise to add to the measurement.
    y(:,k) = C_1*x(:,k) + v;         % Measure the state.
    
    if ismember(k,error_ks)
        y(:,k) = C_1*x_post(:,k-1)*threshold*2; 
    end
    
    L = P_prior * C_1' * inv( C_1 * P_prior *C_1' + R_1);
    x_post(:,k) = x_prior(:,k) + L*(y(:,k)-C_1*x_prior(:,k));
    P_post = (I-L*C_1)*P_prior;
    P_post = 0.5*(P_post + P_post'); % Make sure that the covariance remains symmetric.
  end
  
  % Update the second sensor
  S2_update_interval = 30;
  if mod(k, S2_update_interval)==0
    v = sqrt(R_2)*randn(1,1);        % Find noise to add to the measurement.
    y(:,k) = C_2*x(:,k) + v;         % Measure the state.
    
    if ismember(k,error_ks)
        y(:,k) = C_2*x_post(:,k-1)*threshold*2; 
    end

    L = P_prior * C_2' * inv( C_2 * P_prior *C_2' + R_2);
    x_post(:,k) = x_prior(:,k) + L*(y(:,k)-C_2*x_prior(:,k));
    P_post = (I-L*C_2)*P_prior;
    P_post = 0.5*(P_post + P_post'); % Make sure that the covariance remains symmetric.
  end
  
end

% Plot results -----------------------------------------------------------------

% END ==========================================================================