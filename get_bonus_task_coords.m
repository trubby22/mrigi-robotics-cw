function [x2, y2, z2] = get_bonus_task_coords(x1, y1)

x1 = double(x1);
y1 = double(y1);

db_start_x = 98.00;
db_length_x = 200.0;
db_length_y = 300.0;
db_left_y = db_length_y / 2.0;
db_incline_deg = 15.0;
db_incline_rad = deg2rad(db_incline_deg);
db_top_z = db_length_x * sin(db_incline_rad);
db_bot_length_x = db_length_x * cos(db_incline_rad);

assert(all(x1 >= 0))
assert(all(x1 <= db_length_y))
assert(all(y1 >= 0))
assert(all(y1 <= db_length_x))

x2 = db_start_x + y1 * cos(db_incline_rad);
y2 = db_left_y - x1;
z2 = (x2 - db_start_x) / db_bot_length_x * db_top_z;

end