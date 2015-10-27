function SendCMD(mcu,ch,freq,phase,ap)

ftw = getftw(freq)';
ptw = getptw(phase)';
cmd = double([double('M'),255,ch,ftw',ptw',ap,255]);
fprintf(mcu,cmd);
out=fscanf(mcu);
flushinput(mcu);

if (strcmp(out(1:3),'ACK'))
    disp('Comm Successful.');
else
    disp('Comm Failed!');
end



end