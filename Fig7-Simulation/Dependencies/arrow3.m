function h = arrow3(point,vector,style,color)
    h = quiver3(point(1),point(2),point(3),vector(1),vector(2),vector(3),style,'color',color);
end

