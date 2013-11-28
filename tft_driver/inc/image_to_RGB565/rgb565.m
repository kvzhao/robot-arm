% generates header file with array of integers in RGB565 format
% name of this array is set in variable "nazov"
clc;
clear all;

nazov = 'mandelbrot4';

a = imread('mandelbrot4.jpg');

r = a(:, :, 1); g = a(:, :, 2); b = a(:, :, 3);

r_565 = bitshift(uint16(r/(255/31)),11);
g_565 = bitshift(uint16(g/(255/63)),5);
b_565 = uint16(b/(255/31));

rgb_565 = bitor(bitor(r_565,g_565),b_565);

rgb_565_reshape = reshape(rgb_565', [], 1);

filename = [nazov '.h'];
h = fopen(filename, 'w');

fprintf(h,'#include <stdint.h>\n\nconst static uint16_t %s[] = {\n',nazov);


for k=1:length(rgb_565_reshape)
    fprintf(h,'0x%s',dec2hex(rgb_565_reshape(k)));
    
    if (k < length(rgb_565_reshape))
        fprintf(h,', ');
    end
    
    if (mod(k,9) == 0)  % jump to new line every 9 numbers
        fprintf(h,'\n');
    end
end

fprintf(h, '};');

fclose(h);