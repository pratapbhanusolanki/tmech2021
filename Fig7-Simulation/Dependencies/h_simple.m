function [y,y_d] = h_simple(ki)
    x = ki;
    c = 11;%  
    y=exp(-(x/c).^2);
    y_d = -2*(x/c^2)*exp(-(x/c)^2);
end