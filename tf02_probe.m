function tf02_probe(r)
    dev = serialdev(r, '/dev/serial0', 115200, 8, 'none', 1);
    dev.Timeout = 0.05;
    for i = 1:10
        b = read(dev, 32, 'uint8');
        fprintf('Read %2d: %d bytes. Nonzeros=%d\n', i, numel(b), nnz(b));
        pause(0.02);
    end
end