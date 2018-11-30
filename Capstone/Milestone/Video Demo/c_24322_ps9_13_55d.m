function c_24322_ps9_13_55d()
    D1 = 0.8;
    D2 = 1.2;
    s = 5.67e-8;
    T1 = 400;
    T2 = 300;
    F12 = @(e1,e2) 1/(1./e1 + ((1-e2)./e2).*(D1/D2)^2);
    q12 = @(e1,e2) pi.*(D1^2).*s.*(T1^4 - T2^4).*F12(e1,e2);
    q12_a = @(e2) q12(0.1,e2);
    q12_b = @(e2) q12(0.5,e2);
    q12_c = @(e2) q12(1.0,e2);
    
    figure();
    hold on
        fplot(q12_a, [0.05,1.0]);
        fplot(q12_b, [0.05,1.0]);
        fplot(q12_c, [0.05,1.0]);
    hold off
    title({'13.55.d - cwcolomb','Heat Transfer between Concentric Spheres of Differring Emissivities'});
    legend('e1 = 0.1','e1 = 0.5','e1 = 1.0');
    xlabel('Outer Sphere Emissivity, $$\epsilon_{2}$$ [W]', 'Interpreter', 'latex');
    ylabel('Heat Transferred, $$q_{12}$$', 'Interpreter', 'latex');
    
    
end % #c_24322_ps9_13_55