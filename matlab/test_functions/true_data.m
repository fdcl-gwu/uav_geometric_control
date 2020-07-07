function [x_ship, x_uav, x_us] = true_data(t)

w = 1;
ship_x = 3 * sin(w*t);
ship_y = 2 * cos(w*t);
ship_z = 0;

uav_wrt_ship_x = 0.1 * cos(5*pi*t);
uav_wrt_ship_y = 0.1 * sin(5*pi*t);
uav_wrt_ship_z = 1.0 * sin(2*t);

uav_x = ship_x + uav_wrt_ship_x;
uav_y = ship_y + uav_wrt_ship_y;
uav_z = ship_z + uav_wrt_ship_z;

x_ship = [ship_x, ship_y, ship_z];
x_uav = [uav_x, uav_y, uav_z];
x_us = [uav_wrt_ship_x, uav_wrt_ship_y, uav_wrt_ship_z];

end