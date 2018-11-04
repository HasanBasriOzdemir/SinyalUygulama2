%%
% 1. Sorunun A Sýkký

clc
clear all
close all
fs = 100e3; %fs=100 kHz
f = 100;     %f=100  Hz
t = -1:1/fs:1;
x1 = sawtooth(2*pi*f*t);
plot(t,x1)
axis([0 0.2 -1.2 1.2])
xlabel('Time (sec)')
ylabel('Amplitude') 
title('Sawtooth Periodic Wave')

%%
% 1. Sorunun B Sýkký

clc
clear all
close all
fs = 10e6; %fs=1 MHz
f = 20;       %f=20 Hz
t = -1:1/fs:1;
x2 = square(2*pi*f*t);
plot(t,x2)
axis([0 0.2 -1.2 1.2])
xlabel('Time (sec)')
ylabel('Amplitude')
title('Square Periodic Wave')

%%
% 1. Sorunun C Sýkký

clc
clear all
close all
fs = 100e3; %fs=100 kHz
t = -1:1/fs:1;
x1 = tripuls(t,100e-3); %geniþlik=100 ms
plot(t,x1)
axis([-0.1 0.1 -0.2 1.2])
xlabel('Time (sec)')
ylabel('Amplitude')
title('Triangular Aperiodic Pulse')

%%
% 1. Sorunun D Sýkký

clc
clear all
close all
fs = 10e3; %fs=10 kHz
t = -1:1/fs:1;
x2 = rectpuls(t,50e-3); %geniþlik=50 ms
plot(t,x2)
axis([-0.1 0.1 -0.2 1.2])
xlabel('Time (sec)')
ylabel('Amplitude')
title('Rectangular Aperiodic Pulse')

%%
% 1. Sorunun E Sýkký

clc
clear all
close all
fs = 10e6; %fs=10 MHz
f = 50e3;
tc = gauspuls('cutoff',50e3,0.5,[],-60); 
t1 = -tc : 1e-6 : tc; 
y1 = gauspuls(t1,50e3,0.6);
plot(t1*1e3,y1)
xlabel('Time (ms)')
ylabel('Amplitude')
title('Gaussian Pulse')

%%
% 1. Sorunun F Sýkký

clc
clear all
close all
fs = 200E9;                    % sample freq
D = [2.5 10 100] * 1e-9;     % pulse delay times
t = 0 : 1/fs : 2500/fs;        % signal evaluation time
w = 2e-9;                      % width of each pulse
yp = pulstran(t,D,@rectpuls,w);
T = 0 : 1/50e3 : 10e-3;
D = [0 : 1/1e3 : 10e-3 ; 0.8.^(0:10)]';
Y = pulstran(T,D,@gauspuls,10E3,.5);
plot(t*1e9,yp);
axis([0 12.5 -0.2 1.2])
xlabel('Time (ns)')
ylabel('Amplitude')
title('Rectangular Train')

%%
% 1. Sorunun G Sýkký

clc
clear all
close all
fs = 50E3;                     % sample freq
D = [2.5 10 17.5]' * 0.6e-3;   % pulse delay times
t = 0 : 1/fs : 2500/fs;        % signal evaluation time
w = 10e3;                      % width of each pulse
yp = pulstran(t,D,@rectpuls,w);
T = 0 : 1/50e3 : 10e-3;
D = [0 : 1/1e3 : 10e-3 ; 0.6.^(0:10)]';
Y = pulstran(T,D,@gauspuls,10E3,.5);
plot(T*1e3,Y)
xlabel('Time (ms)')
ylabel('Amplitude')
title('Gaussian Pulse Train')

%%
% 2. Soru

clc
clear all
close all
N=30; %Unit impulse
n=-N:1:N;
y=[zeros(1,N),ones(1,1),zeros(1,N)];
subplot(6,1,1);
plot(n,y);
ylabel('amplitude');
xlabel('time--->');
title('continuous unit impulse signal');
display(y);
subplot(6,1,2);
stem(n,y,'filled','b');
ylabel('amplitude');
xlabel('number of samples--->');
title('discrete unit impulse signal');
display(y);

t = (-30:1:30)'; %Unit Step
unitstep = zeros(size(t)); 
unitstep(t>=1) = 1; 
subplot(6,1,3);
plot(t,unitstep,'g','linewidth',3) 
ylabel('amplitude');
xlabel('time--->');
title('continuous unit step signal');
display(unitstep);
subplot(6,1,4);
stem(t,unitstep,'g','linewidth',3) 
ylabel('amplitude');
xlabel('time--->');
title('discrete unit step signal');
display(unitstep);

t = (-30:1:30)'; %Unit ramp
ramp = t.*unitstep;
subplot(6,1,5);
plot(t,ramp,'r','linewidth',3) 
ylabel('amplitude');
xlabel('time--->');
title('continuous unit ramp signal');
display(ramp);
subplot(6,1,6);
stem(t,ramp,'r','linewidth',3) 
ylabel('amplitude');
xlabel('time--->');
title('discrete unit ramp signal');
display(ramp);


%%
% 3. Soru

clc
clear all
close all
fs=1e3 %fs=1 kHz
f=20 %20 Hz
t=0:1/fs:5
y1=sin(2*pi*f*t)
subplot(2,1,1)
plot(t,y1,'LineWidth',1)
us2=cos(2*pi*f*t)
subplot(2,1,2)
plot(t,us2,'LineWidth',2)
grid on

%%
% 3. Soru x1[n] ve x2[n] grafikleri

clc 
clear all
close all
n=-72:1:72
fs=1e3 %fs=1 kHz
f=20 %20 Hz
t=0:1/fs:5
y=cos(2*pi*n/36)
subplot(2,1,1)
stem(n,y,'filled','r')
title('x1[n]=cos(2*pi*n/36) Grafiði')
y1=sin(2*pi*n/36)
subplot(2,1,2)
stem(n,y1,'filled','b')
title('x2[n]=sin(2*pi*n/36) Grafiði')

%%
% 4. Soru

clc
clear all
close all
t=-5:1:5;
result=t.*(t.^2+3)
g_even=(g(t)+g(-t))/2
g_odd=(g(t)-g(-t))/2
subplot(2,1,1)
plot(t,g_even,'b') 
title('EVEN')
subplot(2,1,2)
plot(t,g_odd,'y') 
title('ODD')

%%
% 5. Soru

clc
clear all
close all
n=-100:100
y=(0.9.^abs(n)).*sin(2*pi*n/4)
sum(abs(y.^2))

%%
% 6. Soru

clc
clear all
close all
f=1e3 %1 kHz
fs=2e3 %fs=2 kHz
t=0:1/fs:5
y1=sin(2*pi*f*t)
subplot(2,1,1)
plot(t,y1)
fs1=50e3
t_samp1=0:1/10:5
us2=sin(2*pi*f*t_samp1)
subplot(2,1,2)
stem(t_samp1,us2)

clc
clear all
close all
f=1e3  %1 kHz
fs=2e3 %fs=2 kHz
t=0:1/fs:5
y1=cos(2*pi*f*t)
subplot(2,1,1)
plot(t,y1)
fs1=50e3
t_samp1=0:1/10:5
us2=cos(2*pi*f*t_samp1)
subplot(2,1,2)
stem(t_samp1,us2)

%%
% 7. Soru

clc
clear all
close all
n = 0:50;
x = 4+cos(2*pi*n/24);

x0 = downsample(x,2,0);
y0 = upsample(x0,2,0);
subplot(3,1,1)
stem(x)
title('Original Signal')

subplot(3,1,2)
stem(y0,'filled','b')
title('N=2 Örnekleme Aralýðý')

n = 0:50;
x = 4+cos(2*pi*n/24);
x0 = downsample(x,10,0);
y0 = upsample(x0,10,0);
subplot(3,1,1)
stem(x,'filled','g')
title('Original Signal')

subplot(3,1,3)
stem(y0,'filled','y')
title('N=10 Örnekleme Aralýðý')
