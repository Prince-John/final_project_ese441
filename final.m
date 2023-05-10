%% Final Project ESE441
%
%  Author: Prince John



%% Task 2

u =0;
m1=1; m2=0.3; l=0.5; g=9.81;

cartpendulum = @(t,x) [x(3);
                       x(4); 
                       (l*m2*sin(x(2))*x(4)^2 + m2*g*cos(x(2))*sin(x(2)))/(m1+m2*(1 - cos(x(2))^2));
                       -1*(l*m2*cos(x(2))*sin(x(2))*x(4)^2 + (m1 + m2)*g*sin(x(2)))/(l*m1 + l*m2*(1 - cos(x(2))^2))];


[t,x_traj] = ode45(cartpendulum,[0,20],[0;pi+0.1;0;0]);


%%
% Visualizing task 2.

L = 0.3; H = 0.2; % cart dimensions
LB = min(x_traj(:,1))-0.5;
UB = max(x_traj(:,1))+0.5;

for k = 1:length(x_traj)
    clf 
    hold on
    % plot cart
    rectangle('Position', [x_traj(k,1)-0.5*L -H L H], 'FaceColor','b','EdgeColor','b');
    % plot pole
    plot([x_traj(k,1) x_traj(k,1)-3*L*sin(x_traj(k,2)-pi)], [0 3*L*cos(x_traj(k,2)-pi)],'r','LineWidth',2.5);
    % plot floor
    plot([LB UB],[-0.5*L-0.05 -0.5*L-0.05],'k--'); 
    axis equal
    axis([LB UB -1.3 1.3])
    drawnow;
end


%% Task 3
%
%
x_initial = [0;0;0;0];
load('cart_pendulum_input.mat', 'u')

 

t_span = [0:0.01:5]';

[t,x_traj] = ode45(@(t, x) cartpendulum_wrapper(t, x, t_span, [0;u]),t_span,x_initial);

%%
% Visualizing task 3.

L = 0.3; H = 0.2; % cart dimensions
LB = min(x_traj(:,1))-0.5;
UB = max(x_traj(:,1))+0.5;

for k = 1:length(x_traj)
    clf 
    hold on
    % plot cart
    rectangle('Position', [x_traj(k,1)-0.5*L -H L H], 'FaceColor','b','EdgeColor','b');
    % plot pole
    plot([x_traj(k,1) x_traj(k,1)-3*L*sin(x_traj(k,2)-pi)], [0 3*L*cos(x_traj(k,2)-pi)],'r','LineWidth',2.5);
    % plot floor
    plot([LB UB],[-0.5*L-0.05 -0.5*L-0.05],'k--'); 
    axis equal
    axis([LB UB -1.3 1.3])
    drawnow;
end


%% Task 4
% *Linearization*
%
% The A matrix is:

A = [0 0 1 0;
     0 0 0 1;
     0 -(m2/m1)*g 0 0;
     0 (m1+m2)*(g/l) 0 0];




eigenvalues_A = eig(A)
%%
% 
% The B matrix is:

B = [0;0;(1/m1);(1/(l*m1))];

%%
% Automated poleplacement

F = place(A,B,[-1,-2,-3,-4]);

%% Task 5:

eq_point = [0;pi;0;0];

cartpendulum_stablized = @(t,x) [x(3);
                       x(4);
                       (l*m2*sin(x(2))*x(4)^2 - (F*(x-eq_point)) + m2*g*cos(x(2))*sin(x(2)))/(m1+m2*(1 - cos(x(2))^2));
                       -1*(l*m2*cos(x(2))*sin(x(2))*x(4)^2 - (F*(x-eq_point))*cos(x(2))+ (m1 + m2)*g*sin(x(2)))/(l*m1 + l*m2*(1 - cos(x(2))^2))];



%% Task 6:
[t,x_traj] = ode45(cartpendulum_stablized,[0,15],[0;pi+1.15;0;0]);


%[t,x_traj] = ode45(@(t,x) cartpencontrolled(t,x,F,l,m1,m2,g,eq_point),[0,15],[0;pi+0.3;0;0]);
%% 

L = 0.3; H = 0.2; % cart dimensions
LB = min(x_traj(:,1))-0.5;
UB = max(x_traj(:,1))+0.5;

for k = 1:length(x_traj)
    clf 
    hold on
    % plot cart
    rectangle('Position', [x_traj(k,1)-0.5*L -H L H], 'FaceColor','b','EdgeColor','b');
    % plot pole
    plot([x_traj(k,1) x_traj(k,1)-3*L*sin(x_traj(k,2)-pi)], [0 3*L*cos(x_traj(k,2)-pi)],'r','LineWidth',2.5);
    % plot floor
    plot([LB UB],[-0.5*L-0.05 -0.5*L-0.05],'k--'); 
    axis equal
    axis([LB UB -1.3 1.3])

    drawnow;
    
end




%% Combined - self stablizing from bottom.

x_initial = [0;0;0;0];
load('cart_pendulum_input.mat', 'u')
t_span = [0:0.01:5]';

[t_1,x_traj_1] = ode45(@(t, x) cartpendulum_wrapper(t, x, t_span, [0;u]),t_span,x_initial);

%%
% Controller kicks in here

x_initial_controller = x_traj_1(end,:); 

t_span = [5:0.01:10]';
eq_point = [0;pi;0;0];

[t_2,x_traj_2] = ode45(cartpendulum_stablized,t_span,x_initial_controller);

x_traj = [x_traj_1;x_traj_2];
t = [t_1;t_2];

%%

L = 0.3; H = 0.2; % cart dimensions
LB = min(x_traj(:,1))-0.5;
UB = max(x_traj(:,1))+0.5;

for k = 1:length(x_traj)
    clf 
    hold on
    % plot cart
    rectangle('Position', [x_traj(k,1)-0.5*L -H L H], 'FaceColor','b','EdgeColor','b');
    % plot pole
    plot([x_traj(k,1) x_traj(k,1)-3*L*sin(x_traj(k,2)-pi)], [0 3*L*cos(x_traj(k,2)-pi)],'r','LineWidth',2.5);
    % plot floor
    plot([LB UB],[-0.5*L-0.05 -0.5*L-0.05],'k--'); 
    axis equal
    axis([LB UB -1.3 1.3])
    drawnow;
end
