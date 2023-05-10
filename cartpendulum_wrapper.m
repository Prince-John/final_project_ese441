
function x = cartpendulum_wrapper(t, x, t_span, u)
m1=1; m2=0.3; l=0.5; g=9.81;


u = interp1(t_span,u,t);


x = [x(3);
     x(4);
     (l*m2*sin(x(2))*x(4)^2 + u+ m2*g*cos(x(2))*sin(x(2)))/(m1+m2*(1 - cos(x(2))^2));
    -1*(l*m2*cos(x(2))*sin(x(2))*x(4)^2 + u*cos(x(2))+ (m1 + m2)*g*sin(x(2)))/(l*m1 + l*m2*(1 - cos(x(2))^2))];

end