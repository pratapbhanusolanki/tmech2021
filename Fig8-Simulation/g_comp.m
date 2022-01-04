function y = g_comp(x)
    x1 = x(1);
    c1 = 19.84;% vertical elev 
    y1=exp(-(x1/c1).^2);
    x2 = x(2);
    c2 = 24.7;% horizontal rover
    y2=exp(-(x2/c2).^2);
    y = y1*y2;
end