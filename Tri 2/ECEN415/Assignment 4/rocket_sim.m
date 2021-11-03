% Nonlinear rocket simulation.
global g c_d M J L eta k

g = -9.8;   
c_d = 2e-3; 
M = 1000;
J = 20e3;
L = 5;
eta = 1000;
k = 6;

% =========================================================================
% Nonlinear model of the rocket motion.
% =========================================================================
% The state here is defined as in the assignment.
% Note that u_1 is passed as x(8) and u_2 is passed to the solver as x(9)
% (That is, matlab's ODE solver doesn't have separate inputs, so we must
% pass them as elements of the x vector.)
%
% x(2)*abs(x(2)) is used in the drag equation to make sure that it always
% opposes the direction of motion. Note however, that it should actually
% not act only on the vertical velocity, but the total velocity.
%
% Note that the model does not check for ground contact, so will allow the
% rocket to move to negative altitude.

% f = @(t,x) [x(2); 
%             g + x(8)*cos(deg2rad(x(5))) / (M+x(7)) - c_d*x(2)*abs(x(2)); 
%             x(4);
%             x(8)*sin(deg2rad(x(5))) / (M+x(7));
%             x(6);
%             k/(J+L^2*x(7))*x(9);
%             -1/eta*(x(8)+x(9));
%             0;
%             0];
% This includs a better drag model for X(1) < 12km.
% https://en.wikipedia.org/wiki/Density_of_air (exponential approximation).
% c_d is chosen to give a terminal veolocity of about 90 m/s at ground
% level.
f = @(t,x) [x(2); 
            g + x(8)*cos(deg2rad(x(5))) / (M+x(7)) - c_d*x(2)*abs(x(2))*exp(-x(1)/10400); 
            x(4);
            x(8)*sin(deg2rad(x(5))) / (M+x(7));
            x(6);
            k/(J+L^2*x(7))*x(9);
            -1/eta*(x(8)+abs(x(9)));
            0;
            0];
        
%==========================================================================

% Simulation parameters ---------------------------------------------------
global x_0
x_0 = [0   % Altitude [m]
       0;  % Vertical speed [m/s]
       0;  % Horizontal position [m]
       0;  % Horizontal speed [m/s]
       0;  % Rocket angle [deg]
       0;  % Rocket rotational speed [deg/s]
       1000;  % Fuel remaining [kg]
       ]';
x = x_0;


dt = 1;             % Simulation time step
sim_time = 15;      % Length of the simulation
elapsed_time = 0;   % Simulation time tracking
store = [];         % Simulation state storage
t_store = [];

% Begin the simulation
figure(1)
clf;

while (x(1) > -0.01) & (elapsed_time <  sim_time)
    
    if x(7) >  0 % Check whether there is any fuel remaining.
    [u1, u2] = controller_command(elapsed_time, x);
    else
        u1 = 0;
        u2 = 0;
    end
  
  % Add some wind disturbances to the rocket.
  %w1 = randn(1,1) * 2000*exp(-(1+x(1))/10400);
  %w2 = randn(1,1) * 1000*exp(-(1+x(1))/10400);
    
  x(8) = u1; % Load the inputs as "fake" state variables for the solver.
  x(9) = u2; 
  
  [t,x] = ode45(f, [0 dt], x);
  
  plot_telemetry(elapsed_time+t, x);

  % You can use these variables later to examine the flight.
  t_store = [t_store; elapsed_time+t];
  store = [store; x];

  elapsed_time = elapsed_time + dt;
  x = x(end,:);
end

x_store = store(:,1:7);
u_store = store(:,8:9);
 
figure(2)
subplot(2,2,[1 2])
plot(t_store, x_store)
ylabel('x')
% legend('Altitude', 'V Velocity', 'Distance', 'H Velocity', 'Pitch', 'Pitch Rate', 'Fuel','Location','eastoutside')

subplot(2,2,3)
plot(t_store, u_store(:,1))
xlabel('Time [s]')
ylabel('u_1')
subplot(2,2,4)
plot(t_store, u_store(:,2))
xlabel('Time [s]')
ylabel('u_2')

%--------------------------------------------------------------------------
% Plotting function to display the telemetry during flight.
%--------------------------------------------------------------------------

function[] = plot_telemetry(t,x)
  global x_0

  subplot(1,2,1)
  xlabel('Distance downrange [m]')
  ylabel('Altitude [m]')
  hold on;
  plot(x(:,3), x(:,1), 'b')
  
  subplot(4,2,4)
  xlabel('Time [s]')
  ylabel('Velocities [m/s]')
  hold on
  plot(t, x(:,2), 'b', t, x(:,4), 'r')
%   legend('Vertical speed', 'Horizontal speed', 'Location', 'northwest')
  
  subplot(4,2,6)
  xlabel('Time [s]')
  ylabel('Pitch angle [deg]')
  hold on
  plot(t, x(:,5), 'b')
  
  subplot(4,2,8)
  xlabel('Time [s]')
  ylabel('Fuel [kg]')
  hold on
  plot(t, x(:, 7), 'b')
  if x(:,7)<0
    set(gca, 'Color' ,'r')
  end
  
  subplot(4,2,2)
  plot(0,0,'bo')
  box off
  hold on
  axis equal
  axis off
  max_u1 = 1000/0.001/15;
  line([0, max_u1*sin(deg2rad(x(end,5)))], [0 max_u1*cos(deg2rad(x(end,5)))],'Color','b')
  line([0, -x(end,8)*sin(deg2rad(x(end,5)))], [0 -x(end,8)*cos(deg2rad(x(end,5)))],'Color','r')
  plot(-x(end,8)*sin(deg2rad(x(end,5))),-x(end,8)*cos(deg2rad(x(end,5))),'r*')
  hold off
  
  drawnow
  
end



%--------------------------------------------------------------------------
% Static open loop controller that simply sets each input to pre-defined
% values at given time steps
%--------------------------------------------------------------------------
function[u1, u2] = static_control(t)
    if t < 6
        u1 = 92000;
        u2 = 0;
    elseif t < 12
        u1 = 55000;
        u2 = 1790;
    else 
        u1 = 22000;
        u2 = -3000;
    end
end

%--------------------------------------------------------------------------
% Linerised open loop controller. Uses Jacobians to form a linerised model
% of the system at each time step. and uses this model to calculate the
% required input to reach a provided target state
%--------------------------------------------------------------------------
function[u1, u2] = open_loop_linearisation(A, B, cur_x, t)
    
    % Target state of the rocket
    if t <= 5
        target_x = [(t+1)*1500/15, 300, 0, 0, 0, 0, 1000-((t+1)*(1000/15))]';
    else
        target_x = [(t+1)*1500/15, 300, 50, 5, 10, 4-((t+1)*(4/10)), 1000-((t+1)*(1000/15))]';
    end
    
    % Calculate the inputs required to achieve the desired state
    u = pinv(B)*(target_x - cur_x - A*cur_x);

    % Clamp the calculated inputs
    if u(1,:) < 0
        u1 = 0;
    else
        u1 = u(1,:);
    end

    if u(2,:) < 0
        u2 = 0;
    else
        u2 = u(2,:);
    end
end


function[u1, u2] = closed_loop(A, B, cur_x, cur_u, t)
    
    load("x_target.mat");
    target_x = x_target(t+1,:)';

    x_bounds = [0.1; 0.16; 1; 0.1; 0.05; 0.05; 16];
    u_bounds = [7.8; 50];

    Q = diag((x_bounds.^2)./((target_x+1e-9).^2));
    R = diag(u_bounds./(cur_u+1e-6).^2); % Ensure that the current u1 != 0

    [K,~,~] = lqr(A, B, Q, R);
    u = -K*(cur_x - target_x);
    
    u1 = min(max(u(1, :), 1e-3), 100000);
    u2 = min(max(u(2, :), -10000), 10000);
end


%--------------------------------------------------------------------------
% Controller code
%--------------------------------------------------------------------------
% You should change the content of this function. The example code included
% here is not particularly clever, but should run.

function[u1, u2] = controller_command(t, x)
    global g c_d M J L eta k linearisation open_loop_control
    persistent Df_x Df_u 
    % % Linerisation ------------------------------------------------------
    % Calculate the Jacobain approximation on the first time step 
    if t == 0
        % Set the initial conditions of the system
        x = [x, 50040, 0]; 
        
        syms x1 x2 x3 x4 x5 x6 x7 u1 u2
        f_sym = [x2; 
                 g + u1*cos(x5) / (M+x7) - c_d*x2^2;  
                 x4;
                 u1*sin(x5) / (M+x7);
                 x6;
                 k/(J+L^2*x7)*u2;
                 -1/eta*(u1+abs(u2));
                 ];
        
        % Generate the jacobian matrecies
        Df_x = jacobian(f_sym, [x1 x2 x3 x4 x5 x6 x7]);
        Df_u = jacobian(f_sym, [u1 u2]);
    end
    % % Linerisation ------------------------------------------------------ 

    % Current state of the rocket    
    cur_x = x(:,1:7)'; % current state of x 
    cur_u = x(:,8:9)'; % current state of u

    % If using a linerised controller, substitute the current operating point
    % into the Jacobian A and B
    if linearisation        
        syms x1 x2 x3 x4 x5 x6 x7 u1 u2
        A = double(subs(Df_x, [x1 x2 x3 x4 x5 x6 x7 u1 u2], x));
        B = double(subs(Df_u, [x1 x2 x3 x4 x5 x6 x7 u1 u2], x));
        
        % Select the controller type to use (This is done externally)
        if open_loop_control
            [u1, u2] = open_loop_linearisation(A, B, cur_x, t);
        else
            [u1, u2] = closed_loop(A, B, cur_x, cur_u, t);
        end
    
    % Use an open loop fuzzy logic static controller
    elseif open_loop_control
        [u1, u2] = static_control(t);
    end    
end

% END =========================================================================