
% https://stumejournals.com/journals/mtm/2020/5/186.full.pdf
% https://iel.ucdavis.edu/design/fourbar/transAngle.html
% https://www.geogebra.org/classic/BueCG9ch
function [theta4_open, theta4_crossed] = four_bar_linkage_theta2_to_theta4(base, driver, coupler, follower, theta1, theta2)
    % Calculate the intermediate terms
    theta2 = theta2 - theta1;
    theta2 = mod(theta2 + pi, 2*pi) - pi;
    P_1 = -2 * driver * follower * sin(theta2);  % First part of the numerator
    P_2 = 2 * follower * (base - driver * cos(theta2));  % Second part of the numerator
    P_3 = (base*base) + (driver*driver) - (coupler*coupler) + (follower*follower) - 2 * base * driver * cos(theta2);  % Denominator
    
    % Check the discriminant for real solutions
    discriminant = (P_1*P_1) + (P_2*P_2) - (P_3*P_3);
    if discriminant < 0 || (P_3 == P_2)
        theta4_open = NaN;
        theta4_crossed = NaN;
        return;
    end
    
    % Calculate the angles for both open and crossed configurations
    theta4_crossed = 2 * atan((-P_1 + sqrt(discriminant)) / (P_3 - P_2));
    theta4_open = 2 * atan((-P_1 - sqrt(discriminant)) / (P_3 - P_2));

    theta4_crossed = theta4_crossed + theta1;
    theta4_open = theta4_open + theta1;

    theta4_open = mod(theta4_open + pi, 2*pi) - pi;
    theta4_crossed = mod(theta4_crossed + pi, 2*pi) - pi;
end
