import math, struct


def f32(x):
    return struct.unpack('f', struct.pack('f', x))[0]


def ident(p):
    x, y = f32(p[0]), f32(p[1])
    # tf2 identity: row.dot(v) + origin
    return (1.0 * x + 0.0 * y + 0.0 * 0.0 + 0.0, 0.0 * x + 1.0 * y + 0.0 * 0.0 + 0.0)


def inside(poly, pt):
    # Polygon::isPointInside of nav2_collision_monitor/src/polygon.cpp
    n = len(poly)
    res = False
    i = n - 1
    for j in range(n):
        if (pt[1] <= poly[i][1]) == (pt[1] > poly[j][1]):
            den = poly[j][1] - poly[i][1]
            num = (pt[1] - poly[i][1]) * (poly[j][0] - poly[i][0])
            if den == 0:
                q = math.nan if (num == 0 or math.isnan(num)) else math.copysign(math.inf, num) * math.copysign(1, den)
            else:
                q = num / den
            if poly[i][0] + q > pt[0]:
                res = not res
        i = j
    return res


box = [(1, 1), (1, -1), (-1, -1), (-1, 1)]
nan = math.nan
pts = [(0.0, 0.0), (0.5, 0.0), (-0.5, 0.0), (-5.0, 0.0), (5.0, 0.0)]
for k in range(4):
    for axis in (0, 1):
        msg = [list(v) for v in box]
        msg[k][axis] = nan
        poly = [ident(v) for v in msg]
        print("NaN at vertex", k, "axis", "xy"[axis], {p: inside(poly, p) for p in pts})
