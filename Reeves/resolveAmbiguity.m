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
