clc;

% Load the data
raw = load('data.txt');
data = raw(1:45,1:2);

%let's split the first 45 value pairs to the three agents:
d1 = data(1:15,1:2);
d2 = data(16:30,1:2);
d3 = data(31:45,1:2);

disp('Correlation:');
corr(data)

% very strong linear correlation -> line fit justified

% Extract size, price data
s1 = d1(:,1);
p1 = d1(:,2);
s2 = d2(:,1);
p2 = d2(:,2);
s3 = d3(:,1);
p3 = d3(:,2);

% Closed form solution
A = [data(:,1) ones(length(data(:,1)),1)];
c_analytical = inv(A'*A)*A'*data(:,2)

%figure
%plot(data(:,1),data(:,2),'*')
%hold on

%syms x
%fplot(c_analytical(1)*x+c_analytical(2))

%% Consensus-based distributed algorithm

P = [1/3 1/3 1/3;1/3 1/3 1/3;1/3 1/3 1/3]; % all to all connection

% Already standardized

A1 = [s1 ones(length(s1),1)];
gradient1 = @(c1,c2) -2*A1'*(p1-A1*[c1;c2]);
A2 = [s2 ones(length(s2),1)];
gradient2 = @(c1,c2) -2*A2'*(p2-A2*[c1;c2]);
A3 = [s3 ones(length(s3),1)];
gradient3 = @(c1,c2) -2*A3'*(p3-A3*[c1;c2]);

H = 500;
e = 1/k; %diminishing step size

ca1 = zeros(2,H); %tabula rasa for results
ca2 = zeros(2,H); %tabula rasa for results
ca3 = zeros(2,H); %tabula rasa for results

c(1:2,1) = [1;1];

for k = 1:H-1
    ca1(1:2,k+1) = (.3333*ca1(1:2,k)+.3333*ca2(1:2,k)+.3333*ca3(1:2,k)) - e * gradient1(ca1(1,k),ca1(2,k));
    ca2(1:2,k+1) = (.3333*ca1(1:2,k)+.3333*ca2(1:2,k)+.3333*ca3(1:2,k)) - e * gradient2(ca2(1,k),ca2(2,k));
    ca3(1:2,k+1) = (.3333*ca1(1:2,k)+.3333*ca2(1:2,k)+.3333*ca3(1:2,k)) - e * gradient3(ca3(1,k),ca3(2,k));
end

c11 = ca1(1,:);
c12 = ca1(2,:);
c21 = ca2(1,:);
c22 = ca2(2,:);
c31 = ca3(1,:);
c32 = ca3(2,:);

figure
plot(1:H,c11)
hold on
plot(1:H,c21)
plot(1:H,c31)
hold off
yline(c_analytical(1))
title("c1 convergence")

figure
plot(1:H,c12)
hold on
plot(1:H,c22)
plot(1:H,c32)
yline(c_analytical(2))
title("c2 convergence")

c1_numerical = [c11(end);c12(end)];
c2_numerical = [c21(end);c22(end)];
c3_numerical = [c31(end);c32(end)];
disp('numerical c values for agent 1');
disp(c1_numerical);
disp('numerical c values for agent 2');
disp(c2_numerical);
disp('numerical c values for agent 3');
disp(c3_numerical);

error_between_numerical_analytical_agent1 = c1_numerical - c_analytical
error_between_numerical_analytical_agent2 = c2_numerical - c_analytical
error_between_numerical_analytical_agent3 = c3_numerical - c_analytical