
fs=44100;
t=[0:1/fs:0.2];
f=123;

y=sin(2*pi*f*t);

soundsc(y,fs);
str = [num2str(f), '.wav'];
wavwrite(y,fs,32,str);



