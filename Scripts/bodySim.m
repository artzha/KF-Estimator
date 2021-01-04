s = serialport("/dev/tty.usbmodem11203", 115200);
configureCallback(s, "off");
configureTerminator(s,"CR/LF");
old_roll = 0;
old_yaw  = 0;
old_pitch= 0;
while 1
    source = readline(s);
    if (strlength(source) > 0)
        orientation = split(source);
        roll = str2double(orientation(1));
        pitch = str2double(orientation(2));
        yaw = str2double(orientation(3));
        plotCube3Angles(roll, pitch, yaw)
        drawnow %Create a 3d figure and plot in real time
        if (old_roll ~= roll || old_yaw ~=yaw || old_pitch ~= pitch)
            old_roll = roll;
            old_yaw = yaw;
            old_pitch = pitch;
            clf %Clean figure and update it
        end
    end
end

