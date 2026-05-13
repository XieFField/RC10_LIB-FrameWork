import math
import numpy as np
import matplotlib.pyplot as plt


def visualize(
    bezier_line,
    bezier_control_points,
    field_rect,
    forbidden_rect,
    first_hit_index,
    half_size,
    min_field_index,
    min_forbidden_index,
):
    plt.figure()
    plt.plot(bezier_line[:, 0], bezier_line[:, 1], color='red', label='bezier')
    plt.scatter(bezier_control_points[:, 0], bezier_control_points[:, 1], label='control')

    fx0, fy0, fx1, fy1 = field_rect
    plt.plot([fx0, fx1, fx1, fx0, fx0], [fy0, fy0, fy1, fy1, fy0], color='black', label='field')

    rx0, ry0, rx1, ry1 = forbidden_rect
    plt.plot([rx0, rx1, rx1, rx0, rx0], [ry0, ry0, ry1, ry1, ry0], color='orange', label='forbidden')

    if first_hit_index is not None:
        hit_point = bezier_line[first_hit_index]
        plt.scatter(hit_point[0], hit_point[1], color='blue', label='first hit')
        cx, cy = hit_point
        car_rect_x = [cx - half_size, cx + half_size, cx + half_size, cx - half_size, cx - half_size]
        car_rect_y = [cy - half_size, cy - half_size, cy + half_size, cy + half_size, cy - half_size]
        plt.plot(car_rect_x, car_rect_y, color='blue', label='car at hit')

    if min_field_index is not None:
        field_point = bezier_line[min_field_index]
        cx, cy = field_point
        car_rect_x = [cx - half_size, cx + half_size, cx + half_size, cx - half_size, cx - half_size]
        car_rect_y = [cy - half_size, cy - half_size, cy + half_size, cy + half_size, cy - half_size]
        plt.plot(car_rect_x, car_rect_y, color='green', label='car at min field')

    if min_forbidden_index is not None:
        forb_point = bezier_line[min_forbidden_index]
        cx, cy = forb_point
        car_rect_x = [cx - half_size, cx + half_size, cx + half_size, cx - half_size, cx - half_size]
        car_rect_y = [cy - half_size, cy - half_size, cy + half_size, cy + half_size, cy - half_size]
        plt.plot(car_rect_x, car_rect_y, color='purple', label='car at min forbidden')

    plt.axis('equal')
    plt.legend()
    plt.show()


def recursive_bezier(pts, t):
    while True:
        recursive_pts = np.empty(shape=(0, 2))
        for i in np.arange(0, pts.shape[0] - 1):
            pt = (1 - t) * pts[i] + t * pts[i + 1]
            recursive_pts = np.append(recursive_pts, np.expand_dims(pt, axis=0), axis=0)

        pts = recursive_pts
        if len(recursive_pts) == 1:
            break

    return recursive_pts[0]


def rect_intersect(a, b):
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    return not (ax1 < bx0 or ax0 > bx1 or ay1 < by0 or ay0 > by1)


def car_hits_forbidden_or_boundary(center, half_size, field_rect, forbidden_rect):
    cx, cy = center
    car_rect = (cx - half_size, cy - half_size, cx + half_size, cy + half_size)

    fx0, fy0, fx1, fy1 = field_rect
    inside_field = car_rect[0] >= fx0 and car_rect[2] <= fx1 and car_rect[1] >= fy0 and car_rect[3] <= fy1
    if not inside_field:
        return True

    return rect_intersect(car_rect, forbidden_rect)


def car_rect_from_center(center, half_size):
    cx, cy = center
    return (cx - half_size, cy - half_size, cx + half_size, cy + half_size)


def car_to_field_signed_distance(car_rect, field_rect):
    fx0, fy0, fx1, fy1 = field_rect
    left_margin = car_rect[0] - fx0
    right_margin = fx1 - car_rect[2]
    bottom_margin = car_rect[1] - fy0
    top_margin = fy1 - car_rect[3]
    return min(left_margin, right_margin, bottom_margin, top_margin)


def rect_signed_distance(a, b):
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b

    dx = max(bx0 - ax1, ax0 - bx1, 0.0)
    dy = max(by0 - ay1, ay0 - by1, 0.0)
    if dx > 0.0 or dy > 0.0:
        return math.hypot(dx, dy)

    overlap_x = min(ax1, bx1) - max(ax0, bx0)
    overlap_y = min(ay1, by1) - max(ay0, by0)
    return -min(overlap_x, overlap_y)


if __name__ == '__main__':
    control_points = np.array([(1.2, 2.6), (0.4, 2.4), (0.6, 3.2)])
    field_rect = (0.0, 0.0, 6.0, 9.45)
    forbidden_rect = (1.2, 3.2, 4.8, 8.0)
    car_size = 0.98
    half_size = car_size / 2.0

    recursive_bezier_line = []
    hit_mask = []
    min_field_distance = float('inf')
    min_forbidden_distance = float('inf')
    min_field_t = None
    min_forbidden_t = None
    min_field_index = None
    min_forbidden_index = None

    for t in np.arange(0.0, 1.001, 0.005):
        pt = recursive_bezier(control_points, t)
        recursive_bezier_line.append(pt)
        car_rect = car_rect_from_center(pt, half_size)
        field_distance = car_to_field_signed_distance(car_rect, field_rect)
        forbidden_distance = rect_signed_distance(car_rect, forbidden_rect)

        if field_distance < min_field_distance:
            min_field_distance = field_distance
            min_field_t = t
            min_field_index = len(recursive_bezier_line) - 1
        if forbidden_distance < min_forbidden_distance:
            min_forbidden_distance = forbidden_distance
            min_forbidden_t = t
            min_forbidden_index = len(recursive_bezier_line) - 1

        hit = car_hits_forbidden_or_boundary(pt, half_size, field_rect, forbidden_rect)
        hit_mask.append(hit)
        if hit:
            print(f"Hit at t={t:.2f}, center=({pt[0]:.3f}, {pt[1]:.3f})")

    bezier_lines = np.stack(recursive_bezier_line, axis=0)
    hit_mask = np.array(hit_mask, dtype=bool)
    first_hit_index = int(np.argmax(hit_mask)) if np.any(hit_mask) else None
    visualize(
        bezier_lines,
        control_points,
        field_rect,
        forbidden_rect,
        first_hit_index,
        half_size,
        min_field_index,
        min_forbidden_index,
    )

    print(
        "Min distance to field boundary: "
        f"{min_field_distance:.3f} m at t={min_field_t:.3f}"
    )
    print(
        "Min distance to forbidden zone: "
        f"{min_forbidden_distance:.3f} m at t={min_forbidden_t:.3f}"
    )
