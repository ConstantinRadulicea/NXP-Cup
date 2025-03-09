function [theta2_open, theta2_crossed] = four_bar_linkage_theta4_to_theta2(base, driver, coupler, follower, theta1, theta4)
theta4_local = pi - theta4 + (theta1);
theta4_local = mod(theta4_local + pi, 2*pi) - pi;
[theta2_open, theta2_crossed] = four_bar_linkage_theta2_to_theta4(base, follower, coupler, driver, 0, theta4_local);
    if ~isnan(theta2_open)
        theta2_open = pi-theta2_open + (theta1);
        theta2_open = mod(theta2_open + pi, 2*pi) - pi;
    end
    
    if ~isnan(theta2_crossed)
        theta2_crossed = pi-theta2_crossed + (theta1);
        theta2_crossed = mod(theta2_crossed + pi, 2*pi) - pi;
    end
end
