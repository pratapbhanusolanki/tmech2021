function [m,s] = compute_array_mean_and_std(m_array,s_array,n)
    n1 = n;
    n2 = n;
    m = m_array(1);
    s = s_array(1);
    for i=2:length(m_array)
        [m,s] = combined_mean_and_std(m,m_array(i),s,s_array(i),n1,n2);
        n1 = n1+n2;
    end
end
