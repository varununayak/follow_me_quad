k=0.15*pi/180; %premultiplier =0.19 as per raksha thesis and 0.12 as per linear interpolation of pwm vs theta; 
sampletime=0.000230;
quad=tf(981*k,[1 0 0]); %g = 981 cm/s2


