function dist_m = read_range(u, BYTES_PER_SAMPLE)
dist_m = NaN;
if u.NumBytesAvailable >= BYTES_PER_SAMPLE
    raw = read(u, u.NumBytesAvailable, "uint8" );

    lastSample = raw(end-BYTES_PER_SAMPLE+1:end);

    dist_m = typecast(uint8(lastSample), "single");

    fprintf("Distance: %.3f m\n", dist_m);

else
    fprintf("No Data Here :(");
end
pause(0.01);
end

