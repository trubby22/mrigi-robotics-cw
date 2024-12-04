function res = approach(prev, safe_z, xyz1)
    [x1, y1, z1] = deal3(xyz1);
    res = [
        prev;
        [x1, y1, safe_z];
        [x1, y1, z1];
    ];
end