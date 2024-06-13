
clear all

syms x

f1 = (1-x)^2;
f2 = (2-x)^2;
f3 = (3-x)^2;
f4 = (4-x)^2;

g1 = diff(f1,x);
g2 = diff(f2,x);
g3 = diff(f3,x);
g4 = diff(f4,x);

g1 = matlabFunction(g1);
g2 = matlabFunction(g2);
g3 = matlabFunction(g3);
g4 = matlabFunction(g4);

% Discrete time consensus setup

% all to all communication
A = ones(4)-eye(4);
D = 3*eye(4);

L = D-A;

% Educated choice of sampling time:
epsilon = 1/max(max(D));

P = (eye(4)-epsilon*L);


H = 50;
x_ = zeros(4,H);

% Diminishing stepsize required! Here we use 1/k.

for k = 1:H-1
    % Consensus + gradient descent
    x_(1:4,k+1) = P*x_(1:4,k) - 1/k * [g1(x_(1,k)) ;g2(x_(2,k));g3(x_(3,k));g4(x_(4,k))];
end

figure
plot(1:H,x_)
legend("Agent1","Agent2","Agent3","Agent4")
title("Converging to a shared decision variable")


% Check if correct
solve(gradient(f1+f2+f3+f4)==0)

% Optimal value is 2.5

% with cost
subs(f1+f2+f3+f4,x,2.5)

% add more steps (H) and you can see the distributed protocol produces same
% value
x_opt = mean(x_(1:4,H))


% cost function analysis:

% f1 = (1-x_(1,:)).^2;
% f2 = (2-x_(2,:)).^2;
% f3 = (3-x_(3,:)).^2;
% f4 = (4-x_(4,:)).^2;
% 
% final_cost = f1(end)+f2(end)+f3(end)+f4(end)
% 
% figure
% plot(1:H,[f1;f2;f3;f4;f1+f2+f3+f4])




