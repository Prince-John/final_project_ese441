%%
clear
clc
close all

variable_input_intermediate = load('cart_pendulum_input.mat');
u_var = variable_input_intermediate.u;

m1 = 1;
m2 = 0.3;
l = 0.5;
g = 9.81;
counter = 1;

% cartpendulum = @(t,x) [x(3);
%                        x(4);
%                        (l*m2*sin(x(2))*(x(4))^2+u+m2*g*cos(x(2))*sin(x(2)))/(m1+m2*(1-(cos(x(2)))^2));
%                        -(l*m2*cos(x(2))*sin(x(2))*(x(4))^2+u*cos(x(2))+(m1+m2)*g*sin(x(2)))/(l*m1+l*m2*(1-(cos(x(2)))^2));
%                        ];

%[t,x_traj] = ode45(cartpendulum,[0,20],[0;pi+0.1;0;0]);
%[t,x_traj] = ode45(@(t,x)cartpen(t,x,u_var,l,m1,m2,g,counter),[0,5],[0;0;0;0]);

syms w x y z u
A_jacobian = jacobian([y;
                       z;
                       (l*m2*sin(x)*((z)^2)+u+m2*g*cos(x)*sin(x))/(m1+m2*(1-(cos(x))^2));
                       -(l*m2*cos(x)*sin(x)*((z)^2)+u*cos(x)+(m1+m2)*g*sin(x))/(l*m1+l*m2*(1-(cos(x))^2));
                       ],[w,x,y,z]);

B_jacobian = jacobian([y;
                       z;
                       (l*m2*sin(x)*((z)^2)+u+m2*g*cos(x)*sin(x))/(m1+m2*(1-(cos(x))^2));
                       -(l*m2*cos(x)*sin(x)*((z)^2)+u*cos(x)+(m1+m2)*g*sin(x))/(l*m1+l*m2*(1-(cos(x))^2));
                       ],[u]);
      
w = 0;
x = pi;
y = 0;
z = 0;
eq_pt = [0;pi;0;0];

linA = double(subs(A_jacobian));
linB = double(subs(B_jacobian));
eig(linA)

K = [linB,linA*linB,linA*linA*linB, linA*linA*linA*linB];

invK = inv(K);

F = invK(end,:)*(linA+4)*(linA+3)*(linA+2)*(linA+1);
%%


[t,x_traj] = ode45(@(t,x)cartpencontrolled(t,x,F,l,m1,m2,g,eq_pt),[0,20],[0;pi+0.1;0;0])

%% Animation
close all

L = 0.3; H = 0.2; % cart dimensions

LB = min(x_traj(:,1))-0.5;
UB = max(x_traj(:,1))+0.5;

for k = 1:length(x_traj)

    clf
    
    hold on
    
    % plot cart
    rectangle('Position', [x_traj(k,1)-0.5*L -H L H],'FaceColor','b','EdgeColor','b');   
    
    % plot pole
    plot([x_traj(k,1) x_traj(k,1)-3*L*sin(x_traj(k,2)-pi)],[0 3*L*cos(x_traj(k,2)-pi)],'r','LineWidth',2.5);
    
    % plot ground
    plot([LB UB],[-0.5*L-0.05 -0.5*L-0.05],'k--');

    axis equal
    axis([LB UB -1.3 1.3])
    
    drawnow;
end

function dxdt = cartpen(t,x,u_var,l,m1,m2,g,counter)
    del_t = 0.01;
    idx = ceil(t/del_t);
    if t == 0
        idx = 1;
    end
    the_t(counter) = t;
    the_idx(counter) = idx;
    the_u(counter) = u_var(idx);
    counter = counter + 1;
    
    if (t==5)
        disp(the_t)
        
        disp(the_u)
        
        disp(the_idx)
    end
    
    dxdt = [x(3);
           x(4);
           (l*m2*sin(x(2))*(x(4))^2+u_var(idx)+m2*g*cos(x(2))*sin(x(2)))/(m1+m2*(1-(cos(x(2)))^2));
           -(l*m2*cos(x(2))*sin(x(2))*(x(4))^2+u_var(idx)*cos(x(2))+(m1+m2)*g*sin(x(2)))/(l*m1+l*m2*(1-(cos(x(2)))^2));
           ];
end

