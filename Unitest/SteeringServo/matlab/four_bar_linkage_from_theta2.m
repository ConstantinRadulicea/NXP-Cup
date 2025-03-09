function [theta3, theta4_open, theta4_crossed] = four_bar_linkage_from_theta2(base, driver, coupler, follower, theta1, theta2)
    % Compute the distance D between the fixed pivots (driver and coupler link)
    D = sqrt(base^2 + driver^2 - 2 * base * driver * cos(theta2 - theta1));
    
    % Check if a real solution exists for theta3 and theta4
    if D > (coupler + follower) || D < abs(coupler - follower)
        theta3 = NaN;
        theta4_open = NaN;
        theta4_crossed = NaN;
        return;
    end
    
    % Compute theta3 using the law of cosines
    cos_theta3 = (base^2 + coupler^2 - D^2) / (2 * base * coupler);
    if abs(cos_theta3) > 1
        theta3 = NaN;
        theta4_open = NaN;
        theta4_crossed = NaN;
        return;
    end
    theta3 = acos(cos_theta3);  % Angle of the coupler relative to the base
    
    % Compute theta4 using the law of cosines
    cos_theta4 = (D^2 + follower^2 - coupler^2) / (2 * D * follower);
    if abs(cos_theta4) > 1
        theta4_open = NaN;
        theta4_crossed = NaN;
        return;
    end
    
    theta4_open = theta2 + acos(cos_theta4);  % Open configuration
    theta4_crossed = theta2 - acos(cos_theta4); % Crossed configuration
end
