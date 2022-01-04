function [y,y_d] = g_simple(ki)
    x = ki;
    c = 15;   %12.21 for h
    y=exp(-(x/c).^2);
    y_d = -2*(x/c^2)*y;
end

