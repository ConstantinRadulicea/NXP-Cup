function [theta2_max, theta2_min, theta3_max, theta3_min, theta4_max, theta4_min] = four_bar_linkage_max_angles(base, driver, coupler, follower, theta1)
    % Compute the extreme values of D
    D_max = base + driver;
    D_min = abs(base - driver);

    % Compute theta2 max and min
    cos_theta2_max = (driver^2 + D_min^2 - coupler^2) / (2 * driver * D_min);
    cos_theta2_min = (driver^2 + D_max^2 - coupler^2) / (2 * driver * D_max);

    if abs(cos_theta2_max) <= 1
        theta2_max = theta1 + acos(cos_theta2_max);
    else
        theta2_max = NaN;
    end

    if abs(cos_theta2_min) <= 1
        theta2_min = theta1 + acos(cos_theta2_min);
    else
        theta2_min = NaN;
    end

    % Compute theta3 max and min
    cos_theta3_max = (driver^2 + coupler^2 - D_min^2) / (2 * driver * coupler);
    cos_theta3_min = (driver^2 + coupler^2 - D_max^2) / (2 * driver * coupler);

    if abs(cos_theta3_max) <= 1
        theta3_max = acos(cos_theta3_max);
    else
        theta3_max = NaN;
    end

    if abs(cos_theta3_min) <= 1
        theta3_min = acos(cos_theta3_min);
    else
        theta3_min = NaN;
    end

    % Compute theta4 max and min
    cos_theta4_max = (D_max^2 + follower^2 - coupler^2) / (2 * D_max * follower);
    cos_theta4_min = (D_min^2 + follower^2 - coupler^2) / (2 * D_min * follower);

    if abs(cos_theta4_max) <= 1
        theta4_max = theta1 + acos(cos_theta4_max);
    else
        theta4_max = NaN;
    end

    if abs(cos_theta4_min) <= 1
        theta4_min = theta1 + acos(cos_theta4_min);
    else
        theta4_min = NaN;
    end
end
