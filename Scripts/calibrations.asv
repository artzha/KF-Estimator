%% Part 1 sample points
s = serialport("/dev/tty.usbmodem11203", 115200);
configureCallback(s, "off");
configureTerminator(s,"CR/LF");
NUM_SAMPLES = 1000;
acc = zeros(NUM_SAMPLES, 3);
i = 1;

while i <= NUM_SAMPLES
    source = readline(s);
    if (size(source) > 0)
        acceleration = split(source);
        x = str2double(acceleration(1))/10000.0; % scale factor for decimal
        y = str2double(acceleration(2))/10000.0;
        z = str2double(acceleration(3))/10000.0;
        acc(i, 1) = x;
        acc(i, 2) = y;
        acc(i, 3) = z;
        i = i + 1;
    end
end

save acc_samples.dat acc -ascii

%% Part 2 fit points
acc=load('acc_samples.dat');
acc_x = acc(:,1);
acc_y = acc(:,2);
acc_z = acc(:,3);

figure(1);
hold on
histfit(acc_x);
pdx = fitdist(acc_x, 'Normal')
title("acceleration in x");
xlabel("acceleration in gs");
hold off

hold on
figure(2);
histfit(acc_y);
pdy = fitdist(acc_y, 'Normal')
title("acceleration in y");
xlabel("acceleration in gs");
hold off

figure(3);
hold on
histfit(acc_z);
pdz = fitdist(acc_z, 'Normal')
title("acceleration in z");
xlabel("acceleration in gs");
hold off
