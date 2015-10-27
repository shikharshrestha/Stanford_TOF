function ptw=getptw(phase)
% % Only valid at fs = 300Mhz
val = uint16((2^14)*(phase)/360);

ptw = typecast(val, 'uint8');
ptw = fliplr(ptw);
end