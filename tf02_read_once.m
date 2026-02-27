function [dist_cm, strength, temp_C] = tf02_read_once(r, opts)
% Robust persistent TF02-Pro UART reader on /dev/serial0 (115200 8N1).
% Keeps a rolling buffer, primes on first open, and returns the latest valid frame.
%
% Usage:
%   [d,s,t] = tf02_read_once(r);
%   [d,s,t] = tf02_read_once(r, struct('debug',true,'primeFrames',8,'chunk',64,'retries',3));
%
% Returns NaNs if no valid frame is found after quick retries.

persistent dev last_r buf primed

% --- defaults ---
if nargin < 2, opts = struct; end
if ~isfield(opts, 'debug'),       opts.debug       = false; end
if ~isfield(opts, 'primeFrames'), opts.primeFrames = 8;     end  % discard N frames at start
if ~isfield(opts, 'chunk'),       opts.chunk       = 64;    end  % read chunk size
if ~isfield(opts, 'retries'),     opts.retries     = 3;     end  % quick retries per call
if ~isfield(opts, 'maxBuf'),      opts.maxBuf      = 512;   end  % rolling buffer cap

dist_cm  = NaN; strength = NaN; temp_C = NaN;

try
    % (Re)open device if needed
    if isempty(dev) || isempty(last_r) || ~isequal(last_r, r)
        dev = serialdev(r, '/dev/serial0', 115200, 8, 'none', 1);
        dev.Timeout = 0.05; % short, snappy
        last_r = r;
        buf = uint8([]);    % reset rolling buffer
        primed = false;
    end

    % One-time priming: read and discard a few frames to sync stream
    if ~primed
        for i=1:opts.primeFrames
            junk = read(dev, opts.chunk, 'uint8'); %#ok<NASGU>
        end
        primed = true;
    end

    % Quick retry loop: read a few chunks and try to parse
    for attempt = 1:opts.retries
        % Append new data to rolling buffer
        newBytes = read(dev, opts.chunk, 'uint8');
        if ~isempty(newBytes)
            buf = [buf; newBytes(:)]; %#ok<AGROW>
            % Cap rolling buffer to avoid unbounded growth
            if numel(buf) > opts.maxBuf
                buf = buf(end-opts.maxBuf+1:end);
            end
        end

        % Search all headers 0x59 0x59 in the buffer
        bytes = double(buf(:).');
        idx = strfind(bytes, [0x59 0x59]);
        if isempty(idx)
            % no header yet; loop for another chunk
            continue;
        end

        % Prefer the latest header that fits a full 9-byte frame
        validStarts = idx(idx + 8 <= numel(bytes));
        if isempty(validStarts)
            continue; % wait for more data to complete the frame
        end
        start = validStarts(end);
        frame = uint8(bytes(start:start+8));

        % Checksum: sum first 8 bytes modulo 256 must equal 9th byte
        cs = uint8(mod(sum(frame(1:8)), 256));
        if cs ~= frame(9)
            % Bad frame; drop prior data up to this header to avoid re-hitting it
            buf = buf(start+1:end);
            continue;
        end

        % Decode TF02 frame
        dist_cm  = double(frame(3)) + bitshift(double(frame(4)), 8);
        strength = double(frame(5)) + bitshift(double(frame(6)), 8);
        temp_raw = double(frame(7)) + bitshift(double(frame(8)), 8);
        temp_C   = temp_raw/8 - 256;

        % Drop data up to this frame so next call biases to newer frames
        buf = buf(start+9:end);

        % Success
        return;
    end

end
end