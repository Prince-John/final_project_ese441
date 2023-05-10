function dxdt = cartpencontrolled(t,x,F,l,m1,m2,g,eq_pt)
    disp(t)
    dxdt = [x(3);
           x(4);
           (l*m2*sin(x(2))*(x(4))^2+(-F*(x-eq_pt))+m2*g*cos(x(2))*sin(x(2)))/(m1+m2*(1-(cos(x(2)))^2));
           -(l*m2*cos(x(2))*sin(x(2))*(x(4))^2+(-F*(x-eq_pt))*cos(x(2))+(m1+m2)*g*sin(x(2)))/(l*m1+l*m2*(1-(cos(x(2)))^2));
           ];
end