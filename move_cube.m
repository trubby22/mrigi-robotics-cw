function res = move_cube(safe_z, xyz1, xyz2)
    [x1, y1, z1] = deal3(xyz1);
    [x2, y2, z2] = deal3(xyz2);
    res = [
        [x1, y1, z1];
        [x1, y1, safe_z];
        [x2, y2, safe_z];
        [x2, y2, z2];
    ];
end