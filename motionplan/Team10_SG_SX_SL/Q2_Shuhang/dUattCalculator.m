function dUatt = dUattCalculator(q, qG, cp, rho_G_star, eta)
    % Calculate the goal position of the control point
    R = [cos(qG(3)), -sin(qG(3)); 
         sin(qG(3)), cos(qG(3))];
    qG_prime = qG(1:2) + R * cp;

    % Calculate the attractive gradient
    rho = norm(q - qG_prime);
    if rho <= rho_G_star
        dUatt = eta * (q - qG_prime);
    else
        dUatt = rho_G_star * eta * ((q - qG_prime) / rho);
    end
end
