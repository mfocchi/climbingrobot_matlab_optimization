
p = [0.3, 2.5, -6]
%p= [0.3, 3, -8]
p = [0.3, 6, -8];


px = p(1);
py = p(2);
pz = p(3);


anchor_pos2 = [0, 5, 0];
zero_rope_length =anchor_pos2(2)/2;

spawn_x = 0.2;

% these are computed according to pinocchio convention but with WF in the
% first anchor point
% first anchor
q0 = [atan2(px-spawn_x, -pz), atan2(-pz, (anchor_pos2(2) - py)), sqrt(px^2+(anchor_pos2(2) -py)^2+pz^2)-zero_rope_length, 0,  -atan2(-pz, (anchor_pos2(2) - py)) , 0, % right anchot joints 
      atan2(px-spawn_x, -pz), -atan2(-pz, py),                   sqrt(px^2+py^2+pz^2)-zero_rope_length,                   0 ,  atan2(-pz, py),                     0]         % left anchor joints

