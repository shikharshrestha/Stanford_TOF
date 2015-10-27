function ftw=getftw(freq)
% % Only valid at fs = 300Mhz
val = uint32((2^32)*(freq)/300000000);

ftw = typecast(val, 'uint8');
ftw = fliplr(ftw);
end