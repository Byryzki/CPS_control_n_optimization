clc;
%time constraints
Ts = 0.01;
Tmax = 8;
time_hist = 0:Ts:Tmax;
n = length(time_hist);
x_hist1 = zeros(n,2);   %γ(h(x)) = h(x)
x_hist2 = zeros(n,2);   %γ(h(x)) = 10h(x)
x_hist3 = zeros(n,2);   %γ(h(x)) = 10h^3(x)
%x_hist(1,:) = [0,0];    %initial value px,py

x0 = [0 0];
xg = [3 3];
lb = [0 0];
ub = [3 3];
obsticle = [1.5 1.8];
r_obsticle = 0.7;
k = 1;
y = 1;

% Define the equality constraints: Aeq*x = beq (none with this task)
Aeq = [];
beq = [];

% Quadratic programming problem update algorithm with γ(h(x)) = h(x)
for i = 1:n-1
    u_gtg = [xg(1,1)-x_hist1(i,1) xg(1,2)-x_hist1(i,2)];
    hx = (x_hist1(i,1)-obsticle(1,1))^2 + (x_hist1(i,2)-obsticle(1,2))^2 - 0.8^2;
    Q = 2*eye(2); % Quadratic coefficients
    c = -2*u_gtg';
    H = -[2*(x_hist1(i,1)-obsticle(1,1)) 2*(x_hist1(i,2)-obsticle(1,2))]; % Coefficients matrix
    b = hx; % Right-hand side vector
    
    [u, fval, exitflag, output] = quadprog(Q, c, H, b, Aeq, beq);
    x_hist1(i+1,:) = [x_hist1(i,1)+Ts*u(1,1) x_hist1(i,2)+Ts*u(2,1)];
    disp(x_hist1(i+1,:));
end

% Quadratic programming problem update algorithm with γ(h(x)) = 10h(x)
for i = 1:n-1
    u_gtg = [xg(1,1)-x_hist2(i,1) xg(1,2)-x_hist2(i,2)];
    hx = (x_hist2(i,1)-obsticle(1,1))^2 + (x_hist2(i,2)-obsticle(1,2))^2 - 0.8^2;
    Q = 2*eye(2); % Quadratic coefficients
    c = -2*u_gtg';
    H = -[2*(x_hist2(i,1)-obsticle(1,1)) 2*(x_hist2(i,2)-obsticle(1,2))]; % Coefficients matrix
    b = 10*hx; % Right-hand side vector
    
    [u, fval, exitflag, output] = quadprog(Q, c, H, b, Aeq, beq);
    x_hist2(i+1,:) = [x_hist2(i,1)+Ts*u(1,1) x_hist2(i,2)+Ts*u(2,1)];
    disp(x_hist2(i+1,:));
end

% Quadratic programming problem update algorithm γ(h(x)) = 10h^3(x)
for i = 1:n-1
    u_gtg = [xg(1,1)-x_hist3(i,1) xg(1,2)-x_hist3(i,2)];
    hx = (x_hist3(i,1)-obsticle(1,1))^2 + (x_hist3(i,2)-obsticle(1,2))^2 - 0.8^2;
    Q = 2*eye(2); % Quadratic coefficients
    c = -2*u_gtg';
    H = -[2*(x_hist3(i,1)-obsticle(1,1)) 2*(x_hist3(i,2)-obsticle(1,2))]; % Coefficients matrix
    b = 10*hx^3; % Right-hand side vector
    
    [u, fval, exitflag, output] = quadprog(Q, c, H, b, Aeq, beq);
    x_hist3(i+1,:) = [x_hist3(i,1)+Ts*u(1,1) x_hist3(i,2)+Ts*u(2,1)];
    disp(x_hist3(i+1,:));
end

% Display the result
disp('Optimal solution:');
disp(u);
disp('Objective function value at optimal solution:');
disp(fval);

% Plot constraints
hold on;
% Define the center and radius of the circular constraint
center = [1.5; 1.8];
radius = 0.7; % Adjust as needed
radius_bigger = radius + 0.1; % Larger radius

% Generate points on the circle
theta = linspace(0, 2*pi, 100);
x_circle = center(1) + radius * cos(theta);
y_circle = center(2) + radius * sin(theta);
x_circle_bigger = center(1) + radius_bigger * cos(theta);
y_circle_bigger = center(2) + radius_bigger * sin(theta);

% Plot the circular constraint
hold on;
plot(3, 3, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'red');       % Goal
plot(1.5, 1.8, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'black'); % Obsticle center
plot(x_circle, y_circle, 'k-', 'LineWidth', 1.5);                   % obsticle
plot(x_circle_bigger, y_circle_bigger, 'k--', 'LineWidth', 1.5);    % Safe zone
plot(x_hist1(:,1), x_hist1(:,2), 'm--', 'LineWidth', 1.5);   %γ(h(x)) = h(x)
plot(x_hist2(:,1), x_hist2(:,2), 'b--', 'LineWidth', 1.5);   %γ(h(x)) = 10h(x)
plot(x_hist3(:,1), x_hist3(:,2), 'g--', 'LineWidth', 1.5);   %γ(h(x)) = 10h^3(x)

% Adjust the axis limits to ensure the circle is visible
axis equal; % Equal aspect ratio
xlim([0, 3]);
ylim([0, 3]);

legend('Goal', 'Obsticle center', 'Obsticle','Safe zone','γ(h(x)) = h(x)','γ(h(x)) = 10h(x)','γ(h(x)) = 10h^3(x)');
hold off;
xlabel('p1');
ylabel('p2');
title('Optimal travel path with obsticle');
