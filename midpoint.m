function [x3, y3] = midpoint(x1, y1, x2, y2, m, n)
    x3 = (m * x2 + n * x1) / (m + n);
    y3 = (m * y2 + n * y1) / (m + n);
end