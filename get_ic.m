function ic = get_ic(b,a,iv)

    n = size(iv,2);
    
    iva = ones(length(a)-1,1)*iv;
    ivb = ones(length(b),1)*iv;
    
    ic = zeros(length(a)-1,n);
    
    for k=1:n
        ic(:,k) = filtic(b,a,iva(:,k),ivb(:,k));
    end

end
