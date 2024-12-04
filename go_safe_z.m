function res = go_safe_z(safe_z, xyz1)
    [x1, y1, z1] = deal3(xyz1);
    res = [
        [x1, y1, z1];
        [x1, y1, safe_z];
    ];
end