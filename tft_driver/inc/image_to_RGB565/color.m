% to find out color code in format RGB565
r = 255; g = 0; b = 0;

r_565 = bitshift(uint16(r/(255/31)),11);
g_565 = bitshift(uint16(g/(255/63)),5);
b_565 = uint16(b/(255/31));

rgb_565 = bitor(bitor(r_565,g_565),b_565);

dec2hex(rgb_565)