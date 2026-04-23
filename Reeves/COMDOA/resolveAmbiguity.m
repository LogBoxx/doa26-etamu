% ======================================================================= %
% Script:   resolveAmbiguity
% Function: 4-element UCA has tendency to attain false azimuth estimations
%           180 degrees across from true azimuth. To correct, script
%           inputs current estimation and adds 180 degrees for comparison
%           to previous estimation. If 180 shift is closer to previous
%           estimation, it is used. Otherwise, current estimation is
%           accepted instead.
% Notes:    
% Author:   Parker Reeves
% Date:     02/13/2026
% ======================================================================= %

function theta_corrected = resolveAmbiguity(theta_est, theta_prev)

% Generate both possible candidates
theta_candidate1 = theta_est;
theta_candidate2 = mod(theta_est + 180, 360);

% Choose the one closest to previous estimate
diff1 = abs(wrapTo180(theta_candidate1 - theta_prev));
diff2 = abs(wrapTo180(theta_candidate2 - theta_prev));

if diff1 < diff2
    theta_corrected = theta_candidate1;
else
    theta_corrected = theta_candidate2;
end

end
