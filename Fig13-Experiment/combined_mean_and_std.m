function [m,s] = combined_mean_and_std(m1,m2,s1,s2,n1,n2)
    m = (n1*m1+n2*m2)/(n1+n2);
    sq = ((n1-1)*s1^2+(n2-1)*s2^2)/(n1+n2-1) + n1*m1*(m1-m2)^2/((n1+n2)*(n1+n2-1));
    s = sqrt(sq);
end

