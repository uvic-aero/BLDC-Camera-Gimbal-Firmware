
max_speed = 720;
min_speed = 2;

max_pulse = 255;
min_pulse = 100;

f = @sqrt;
g = @(x) 1 - exp(-2*x/max_speed);

max_curve_val = feval(g, max_speed);

speed = 0:0.1:max_speed;
pulse = (1/max_curve_val)*(max_pulse - min_pulse)*feval(g, speed) + min_pulse;

plot(speed, pulse);
axis([0 max_speed 0 (max_pulse + 20)]);