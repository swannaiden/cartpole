function E = cartPoleEnergy(x)
    [U,T] = autoGen_cartPoleEnergy(x(1),x(2), x(3), x(4),1 ,1, 9.81, 1);
    E = U+T;
    
end