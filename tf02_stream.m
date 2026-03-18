function [dist_cm, strength] = tf02_stream(r)
disp("Starting TF02 continuous reader...")
while true
    try
        % ---- READ LOOP (stays here until error) ----
        while true
            clear dev;
            dev = serialdev(r, '/dev/serial0', 115200, 8, 'none', 1);
            dev.Timeout = 0.1;

            raw = read(dev, 19, 'uint8');   % burst read

            % Find header 0x59 0x59
            idx = strfind(raw(:).', [hex2dec('59') hex2dec('59')]);
            if isempty(idx)
                continue;   % just try again
            end

            start = idx(1);

            % Check full frame available
            if start + 8 > length(raw)
                continue;   %try again
            end

            rest = raw(start+2:start+8);

            % ---- DECODE ----
            dist_cm = double(rest(1)) + bitshift(double(rest(2)), 8);
            strength = double(rest(3)) + bitshift(double(rest(4)), 8);
            temp_raw = double(rest(5)) + bitshift(double(rest(6)), 8);
            temp_C   = temp_raw/8 - 256;

            fprintf("Dist = %.1f cm   Strength = %d   Temp = %.1f C\n", ...
                dist_cm, strength, temp_C);

            pause(0.001);
            %prevents CPU spin (important)
            clear dev;
        end

    catch exception
        fprintf("Error: %s\n", exception.message);
        clear dev;
        pause(0.5);  
        fprintf("Restarting reader...\n");
    end
end
end