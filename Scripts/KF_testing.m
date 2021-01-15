s = serialport("/dev/tty.usbmodem11303", 115200);
configureCallback(s, "off");
configureTerminator(s,"CR/LF");
NUM_SAMPLES = 1000;
position_data = zeros(NUM_SAMPLES, 3);

i = 1;
while i <= NUM_SAMPLES
    source = readline(s);
    if (strlength(source) > 0)
        position = split(source);
        position_data(i, 1) = str2double(position(1));
        position_data(i, 2) = str2double(position(2));
        position_data(i, 3) = str2double(position(3));
        i = i +1;
    end
end

%% plot position_data

t = linspace(0, 10, NUM_SAMPLES);
hold on
title("Estimated position vs time");
plot(t, position(:,1), '-r');
plot(t, position(:,2), '-g');
plot(t, position(:,3), '-b');
legend("x", "y", "z");
hold off