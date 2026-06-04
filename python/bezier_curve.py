import math
import numpy as np
import matplotlib.pyplot as plt


def visualize(
    bezier_line,
    bezier_control_points,
    field_rect,
    forbidden_rect,
    half_size,
    intersect_idx,
    is_p1_on_bisector,
    p1_proj,
    mid_p0p2,
    dir_p0p2
):
    plt.figure()
    plt.plot(bezier_line[:, 0], bezier_line[:, 1], color='red', label='bezier')
    plt.scatter(bezier_control_points[:, 0], bezier_control_points[:, 1], label='control', zorder=5)

    fx0, fy0, fx1, fy1 = field_rect
    plt.plot([fx0, fx1, fx1, fx0, fx0], [fy0, fy0, fy1, fy1, fy0], color='black', label='field')

    rx0, ry0, rx1, ry1 = forbidden_rect
    plt.plot([rx0, rx1, rx1, rx0, rx0], [ry0, ry0, ry1, ry1, ry0], color='orange', label='forbidden')

    if mid_p0p2 is not None and dir_p0p2 is not None:
        perp_dir = np.array([-dir_p0p2[1], dir_p0p2[0]])
        dir_norm = np.linalg.norm(perp_dir)
        if dir_norm > 1e-9:
            perp_dir = perp_dir / dir_norm
            line_len = 15.0
            p_start = mid_p0p2 - perp_dir * line_len
            p_end = mid_p0p2 + perp_dir * line_len
            plt.plot([p_start[0], p_end[0]], [p_start[1], p_end[1]], color='gray', linestyle='--', label='bisector')

    if not is_p1_on_bisector and p1_proj is not None:
        plt.scatter(p1_proj[0], p1_proj[1], color='magenta', marker='x', s=80, label='projected control', zorder=5)

    if intersect_idx is not None:
        hit_point = bezier_line[intersect_idx]
        plt.scatter(hit_point[0], hit_point[1], color='blue', label='curve intersect', zorder=4)
        cx, cy = hit_point
        car_rect_x = [cx - half_size, cx + half_size, cx + half_size, cx - half_size, cx - half_size]
        car_rect_y = [cy - half_size, cy - half_size, cy + half_size, cy + half_size, cy - half_size]
        plt.plot(car_rect_x, car_rect_y, color='cyan', label='car at intersect')

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
    """  control_points = np.array([(1.2, 2.6), (0.4, 2.4), (0.6, 3.2)]) """
    """control_points = np.array([(1.2, 2.57), (0.35, 2.35), (0.57, 3.2)])"""
    control_points = np.array([(1.0, 1.0), (2.5, 2.0), (2.4, 0.8)])
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

    # Find the intersection between the trajectory and the perpendicular bisector of P0 and P2
    p0 = control_points[0]
    p1 = control_points[1]
    p2 = control_points[2]
    
    mid_p0p2 = (p0 + p2) / 2.0
    dir_p0p2 = p2 - p0
    dir_norm = np.linalg.norm(dir_p0p2)
    normal_p0p2 = dir_p0p2 / dir_norm if dir_norm > 1e-9 else np.array([1.0, 0.0])
    
    p1_dist_to_bisector = np.dot(p1 - mid_p0p2, normal_p0p2)
    is_p1_on_bisector = abs(p1_dist_to_bisector) < 1e-6
    p1_proj = p1 - p1_dist_to_bisector * normal_p0p2
    
    print(f"Control point on perpendicular bisector: {is_p1_on_bisector}")
    if not is_p1_on_bisector:
        print(f"Nearest point on bisector for control point: ({p1_proj[0]:.3f}, {p1_proj[1]:.3f})")

    bezier_lines = np.stack(recursive_bezier_line, axis=0)
    
    # Find the intersection of bezier with the bisector
    dots = np.dot(bezier_lines - mid_p0p2, dir_p0p2)
    bisector_indices = np.where(np.diff(np.sign(dots)))[0]
    
    intersect_idx = None
    if len(bisector_indices) > 0:
        intersect_idx = bisector_indices[0]
        # Approximate intersection point
        pt_intersect = bezier_lines[intersect_idx]
        intersect_rect = car_rect_from_center(pt_intersect, half_size)
        dist_field = car_to_field_signed_distance(intersect_rect, field_rect)
        dist_forb = rect_signed_distance(intersect_rect, forbidden_rect)
        print(f"Trajectory intersects bisector near center=({pt_intersect[0]:.3f}, {pt_intersect[1]:.3f})")
        print(f"At intersection -> Min distance to field boundary: {dist_field:.3f} m")
        print(f"At intersection -> Min distance to forbidden zone: {dist_forb:.3f} m")
    else:
        print("Trajectory does not intersect the perpendicular bisector.")

    visualize(
        bezier_line=bezier_lines,
        bezier_control_points=control_points,
        field_rect=field_rect,
        forbidden_rect=forbidden_rect,
        half_size=half_size,
        intersect_idx=intersect_idx,
        is_p1_on_bisector=is_p1_on_bisector,
        p1_proj=p1_proj,
        mid_p0p2=mid_p0p2,
        dir_p0p2=dir_p0p2
    )