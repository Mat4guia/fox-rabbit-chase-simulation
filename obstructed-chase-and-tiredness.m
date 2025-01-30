function dposfdt = foxode(t, pos_f, vel_r, vel_f)
    % Calculate the position of the rabbit
    pos_r = [vel_r *cos(pi/4)*t; vel_r *sin(pi/4)* t];
    
    % Calculate the distance between the fox and the rabbit
    dist = sqrt((pos_r(1) - pos_f(1))^2 + (pos_r(2) - pos_f(2))^2);
    dist_point = sqrt((200 - pos_f(1))^2 + (-400 - pos_f(2))^2);
    
    % Define the derivative (velocity) of the fox's position
    dposfdt = zeros(2,1);
    
    if (pos_r(1)==pos_f(1)) && pos_f(1)<200
        dposfdt(2) = vel_f;
        dposfdt(1) = 0;
   
    else
        m = (pos_f(2) - pos_r(2))/(pos_f(1) - pos_r(1));
        c = pos_r(2)-m*pos_r(1);
        y_intersection = m*200+c; %y
        if pos_f(2)<-400 && pos_f(1)>200
            dposfdt(1) = vel_f * (200 - pos_f(1)) / dist_point;
            dposfdt(2) = vel_f * (-400 - pos_f(2)) / dist_point;
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
end

function [value, isterminal, direction] = foxrabevents(t, pos_f, vel_r, mindist, burrow)
    % Calculate the position of the rabbit
    pos_r = [vel_r *cos(pi/4)*t; vel_r *sin(pi/4)* t];
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
vel_r = 11;          % Rabbit speed
vel_f = 15;          % Fox speed
pos0_f = [300; -550]; % Initial position of the fox
mindist = 0.1;       % Minimum distance for capture
burrow = [600; 600];  % Position of the rabbit's burrow

% Choose a reasonable time span
ts = [0 1200];  % Increase the time span to give enough time for events to happen

% Set up options with the event function
options = odeset('Events', @(t, pos_f)foxrabevents(t, pos_f, vel_r, ...
    mindist, burrow), 'RelTol', 1e-6, 'AbsTol', 1e-9);
% Solve the ODE
[t, pos_f, t_events, posf_events, event_index] = ode45(@(t, pos_f) ...
    foxode(t, pos_f, vel_r, vel_f), ts, pos0_f, options);

% Plot the fox's path
figure;
hold on;
% Plot the line between the origin_f and origin_r to show that the fox
% never sees the rabbit
%x = [300, 0]; % x-coordinates
%y = [-550, 0]; % y-coordinates
%plot(x, y, '-o'); % '-o' adds markers at the points
plot(pos_f(:,1), pos_f(:,2)); % Fox's path
plot(vel_r*cos(pi/4)*t, vel_r*sin(pi/4)*t); % Rabbit's path
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
disp(posf_events);
distances = sqrt((pos_f(2:end,1) - pos_f(1:end-1,1)).^2 + ...
        (pos_f(2:end,2) - pos_f(1:end-1,2)).^2 );
disp("Total distance travelled by the fox is:");
disp(sum(distances));


if event_index == 1
    disp("The fox ate the rabbit");
end
if event_index == 2
    disp("The rabbit escaped");
end

hold off;
