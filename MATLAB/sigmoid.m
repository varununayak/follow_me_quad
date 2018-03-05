k=0.6;
xref=30;
x=0:0.5:100;
y=1./(1+exp(-k*(x-xref)));
plot(x,y);
grid on
