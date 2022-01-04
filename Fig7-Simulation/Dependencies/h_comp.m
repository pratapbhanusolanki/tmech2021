function y = h_comp(x)
    x1 = x(1); %alpha
    c1 = 11.12;% vertical rover LED 
    y1=exp(-(x1/c1).^2);
    x2 = x(2); %beta
    c2 = 14.52;% horizontal elev LED
    y2=exp(-(x2/c2).^2);
    y = y1*y2;
end