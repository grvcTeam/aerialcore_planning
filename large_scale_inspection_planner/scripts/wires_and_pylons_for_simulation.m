clear; close all; clc
format long g

pylons_position_cartesian = [-409.120 449.015 25;
-430.169 276.424 25;
-450.540 108.275 25;
-470.078 -41.682 25;
-490.208 -215.834 25;
-492.875 -240.864 25;
-598.496 -281.095 20;
-633.009 -319.444 20;
-627.564 -283.028 20;
-611.221 -232.533 20;
-756.512 -346.437 20; # Unappealing in simulation because pylon underneath the ground level. Removed in the world.
-544.794 -296.507 20;
-413.182 -301.398 20;
-244.429 -290.325 30;
-32.435 -276.509 30;
119.719 -383.291 30;
276.420 -493.459 30;
-391.957 -202.912 20;
-301.302 -167.188 20;
-208.036 -131.632 20;
-118.176 -97.355 20;
-28.178 -63.028 20;
65.174 -27.481 20;
62.631 -21.742 20;
163.171 6.176 20;
252.476 37.164 20;
351.502 70.480 20;
447.289 103.004 20;
543.828 136.838 20;
641.898 170.080 20;
719.770 208.398 30;
769.668 108.103 35;
894.348 293.824 25;];

pylons_position_geographic = [38.142767 -3.178503 475;
38.141211 -3.178739 456;
38.139696 -3.178968 440;
38.138344 -3.179188 434;
38.136774 -3.179414 436;
38.136548 -3.179444 437;
38.136184 -3.180648 437;
38.135838 -3.181041 437;
38.136166 -3.180980 437;
38.136621 -3.180794 437;
38.135592 -3.182450 426; # Unappealing in simulation because pylon underneath the ground level. Removed in the world.
38.136046 -3.180035 439;
38.136004 -3.178533 438;
38.136107 -3.176607 440;
38.136235 -3.174189 446;
38.135275 -3.172450 451;
38.134285 -3.170660 465;
38.136892 -3.178293 437;
38.137216 -3.177259 439;
38.137538 -3.176196 440;
38.137848 -3.175171 441;
38.138159 -3.174145 443;
38.138481 -3.173080 445;
38.138533 -3.173109 445;
38.138786 -3.171963 446;
38.139067 -3.170944 447;
38.139369 -3.169815 449;
38.139663 -3.168722 450;
38.139970 -3.167622 452;
38.140271 -3.166503 454;
38.140618 -3.165615 453;
38.139714 -3.165044 454;
38.141390 -3.163625 451;];

pylons_connections_indexes = [ { [2] }
{ [1 3] }
{ [2 4] }
{ [3 5] }
{ [4 6] }
{ [5 7 18] }
{ [6 8] }
{ [7 9 11 12] }
{ [8 10] }
{ [9] }
{ [8] } # Unappealing in simulation because pylon underneath the ground level. Removed in the world.
{ [8 13] }
{ [12 14] }
{ [13 15] }
{ [14 16] }
{ [15 17] }
{ [16] }
{ [6 19] }
{ [18 20] }
{ [19 21] }
{ [20 22] }
{ [21 23] }
{ [22 24 25] }
{ [23] }
{ [23 26] }
{ [25 27] }
{ [26 28] }
{ [27 29] }
{ [28 30] }
{ [29 31] }
{ [30 32 33] }
{ [31] }
{ [31] } ];

map_origin_geo = [38.138728 -3.173825 444];


# Calculate and print the wires position, rotation and length to copy in the world file:
wires=[0 0 0 0 0 0 0];
wires_count = 1;

[N_pylons,~] = size(pylons_position_cartesian);
for it_1=1:N_pylons
  [~,N_connections_current_pylon] = size(pylons_connections_indexes{it_1});
  for it_2=1:N_connections_current_pylon
    if it_1 > pylons_connections_indexes{it_1}(1,it_2)  # Don't repeat twice the connection
      wires(wires_count,:) = [(pylons_position_cartesian(it_1,1)+pylons_position_cartesian(pylons_connections_indexes{it_1}(1,it_2),1))/2 (pylons_position_cartesian(it_1,2)+pylons_position_cartesian(pylons_connections_indexes{it_1}(1,it_2),2))/2 (pylons_position_cartesian(it_1,3)+pylons_position_geographic(it_1,3)+pylons_position_cartesian(pylons_connections_indexes{it_1}(1,it_2),3)+pylons_position_geographic(pylons_connections_indexes{it_1}(1,it_2),3)-8*2)/2-map_origin_geo(1,3) pi/2-atan2((pylons_position_cartesian(it_1,3)+pylons_position_geographic(it_1,3)-pylons_position_cartesian(pylons_connections_indexes{it_1}(1,it_2),3)-pylons_position_geographic(pylons_connections_indexes{it_1}(1,it_2),3)),(sqrt((pylons_position_cartesian(it_1,1)-pylons_position_cartesian(pylons_connections_indexes{it_1}(1,it_2),1))^2+(pylons_position_cartesian(it_1,2)-pylons_position_cartesian(pylons_connections_indexes{it_1}(1,it_2),2))^2))) 0 pi/2+atan2((pylons_position_cartesian(it_1,2)-pylons_position_cartesian(pylons_connections_indexes{it_1}(1,it_2),2)),(pylons_position_cartesian(it_1,1)-pylons_position_cartesian(pylons_connections_indexes{it_1}(1,it_2),1))) sqrt((pylons_position_cartesian(it_1,1)-pylons_position_cartesian(pylons_connections_indexes{it_1}(1,it_2),1))^2+(pylons_position_cartesian(it_1,2)-pylons_position_cartesian(pylons_connections_indexes{it_1}(1,it_2),2))^2+(pylons_position_cartesian(it_1,3)+pylons_position_geographic(it_1,3)-pylons_position_cartesian(pylons_connections_indexes{it_1}(1,it_2),3)-pylons_position_geographic(pylons_connections_indexes{it_1}(1,it_2),3))^2)];
      wires_count = wires_count+1;
    end
  end
end

wires


# Calculate and print the pylons position, rotation and length to copy in the world file:
pylons=[0 0 0 0 0 0 0];
pylons_count = 1;

[N_pylons,~] = size(pylons_position_cartesian);
for it_1=1:N_pylons
  pylons(pylons_count,:)=[pylons_position_cartesian(it_1,1) pylons_position_cartesian(it_1,2) (pylons_position_cartesian(it_1,3)+pylons_position_geographic(it_1,3)-map_origin_geo(1,3)-7)/2 0 0 0 pylons_position_cartesian(it_1,3)+pylons_position_geographic(it_1,3)-map_origin_geo(1,3)-7];
  pylons_count = pylons_count+1;
end

pylons
