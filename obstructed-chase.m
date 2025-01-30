function dstatedt = foxode(~,state, vr_0,mu_r, vf_0,mu_f)
    pos_f = state(1:2);
    distf_travelled=state(3);
    % Calculate the position of the rabbit
    pos_r = state(4:5);
    distr_travelled=state(6);
    % Calculate the distance between the fox and the rabbit
    dist = sqrt((pos_r(1) - pos_f(1))^2 + (pos_r(2) - pos_f(2))^2);
    dist_point = sqrt((200 - pos_f(1))^2 + (-400 - pos_f(2))^2);
    
    % Define the derivative (velocity) of the fox's position
    
    vel_f = vf_0 * exp(-mu_f * distf_travelled);
    vel_r = vr_0 * exp(-mu_r * distr_travelled);
    
    rabbit_direction = [cos(pi/4); sin(pi/4)];
    dposfdt = zeros(2,1);
    
    % Logic for warehouse obstructing the predator's view
    
    if (pos_r(1)==pos_f(1)) && pos_f(1)<200
        dposfdt(2) = vel_f;
        dposfdt(1) = 0;
   
    else
        % Draw line between predator and prey
        m = (pos_f(2) - pos_r(2))/(pos_f(1) - pos_r(1)); 
        c = pos_r(2)-m*pos_r(1);
        
        y_intersection = m*200+c; %y-intersection with warehouse
        % If we are before this position we can never see the prey
        % anyway, and the shortest path is to go to the corner of the
        % warehouse
        if pos_f(2)<-400 && pos_f(1)>200
            dposfdt(1) = vel_f * (200 - pos_f(1)) / dist_point;
            dposfdt(2) = vel_f * (-400 - pos_f(2)) / dist_point;
        % If there is an intersection with the top part of the
        % warehouse we move directly upwards
        % this is the only wall we have to check intersection with
        elseif y_intersection >= -400 && y_intersection <= 0
            if (pos_r(1)>200 && pos_f(1)<200)    
                dposfdt(1) = 0;
                dposfdt(2) = vel_f;
            end
        else
            dposfdt(1) = vel_f * (pos_r(1) - pos_f(1)) / dist;
            dposfdt(2) = vel_f * (pos_r(2) - pos_f(2)) / dist;
        end
    end
        
    dposrdt = vel_r * rabbit_direction;
    d_distf_travelled_dt = vel_f;
    d_distr_travelled_dt = vel_r;
    % Combine derivatives into a single state vector
    dstatedt = [dposfdt; d_distf_travelled_dt; dposrdt; d_distr_travelled_dt];
end

function [value, isterminal, direction] = foxrabevents(~,state, mindist, burrow)
    % Calculate the position of the rabbit
    pos_f = state(1:2);  % Fox's position [x; y]
    pos_r = state(4:5);  % Rabbit's position [x; y]
    dist = sqrt((pos_r(1) - pos_f(1))^2 + (pos_r(2) - pos_f(2))^2) - mindist;
    % Event 1: Fox captures the rabbit (distance becomes less than or equal to mindist)
    value(1) = dist;
    direction = 0;
    isterminal(1) = 1;   % Stop the integration if event occurs
    % Event 2: Rabbit reaches the burrow (distance to burrow becomes zero)
    value(2) = pos_r(1) - burrow(1);
    direction(2) = 1;
    isterminal(2) = 1;    
end

% Main script
vel0_r = 11; % Rabbit speed
vel0_f = 15; % Fox speed
mu_f = 0.0002;
mu_r = 0.0008;
pos0_f = [300; -550]; % Initial position of the fox
pos0_r = [0; 0]; % Initial position of the rabbit
mindist = 0.1;       % Minimum distance for capture
burrow = [600; 600];  % Position of the rabbit's burrow

% Choose a reasonable time span
ts = [0 1500];  % Increase the time span to give enough time for events to happen

state0 = [pos0_f; 0; pos0_r; 0]; % Initial distance traveled is 0
% Set up options with the event function
options = odeset('Events', @(t, state)foxrabevents(t, state, mindist, burrow),'RelTol', 1e-6, 'AbsTol', 1e-9);

% Solve the ODE
[t, state, t_events, state_events, event_index] = ode45(@(t, state)foxode(t,state, vel0_r,mu_r, vel0_f,mu_f),ts, state0, options);
% Plot the fox's path
pos_f = state(:, 1:2);
pos_r = state(:, 4:5);
figure;
hold on;
% Plot the line
%x = [300, 0]; % x-coordinates
%y = [-550, 0]; % y-coordinates
%plot(x, y, '-o'); % '-o' adds markers at the points
plot(pos_f(:,1), pos_f(:,2)); % Fox's path
plot(pos_r(:, 1), pos_r(:, 2)); % Rabbit's path
xlabel('X Position');
ylabel('Y Position');
title('Fox vs Rabbit Chase');
legend('Fox Path', 'Rabbit Path', 'Location', 'Best');

% Add a rectangle (for the area of interest)
rectangle('Position', [200, -400, 400, 400]);
plot(600, 600, 'o', 'MarkerEdgeColor', 'black','HandleVisibility', 'off'); % Burrow
text(600 + 10, 600, 'Burrow', 'FontSize', 12);
% Display the event information
disp('Event Information:');
disp('Event Times:');
disp(t_events);
disp('Event Positions (Fox):');
disp(state_events(1:2));
disp("Total distance travelled by the fox is:");
disp(state(end,3));

if event_index == 1
    disp("The fox ate the rabbit");
end
if event_index == 2
    disp("The rabbit escaped");
end

hold off;
