from params import *

def main(args=None):

    min_angle = FOV_MIN_ANGLE
    max_angle = FOV_MAX_ANGLE
    min_tilt_angle = FOV_MIN_ANGLE + TILT_ANGLE
    max_tilt_angle = FOV_MAX_ANGLE + TILT_ANGLE
    fov = abs(FOV_MIN_ANGLE) + abs(FOV_MAX_ANGLE)
    rays = RAYS
    in_rays = INPUT    
    dist = [FOV_MIN_ANGLE] + DISTRIBUTION + [FOV_MAX_ANGLE]
    
    ranges = []
    ranges_unclipped = []
    for i in range(len(dist) - 1):
        if dist[i] + TILT_ANGLE > 0: break
        if dist[i+1] + TILT_ANGLE > 0:
            ranges.append((dist[i] + TILT_ANGLE, 0))
        else:
            ranges.append((dist[i] + TILT_ANGLE, dist[i+1] + TILT_ANGLE))
        ranges_unclipped.append((dist[i] + TILT_ANGLE, dist[i+1] + TILT_ANGLE))
    if DEBUG: print(f'[INFO] [ranges]: {ranges}\n')
    
    if min_angle < 0:
        clip_fov = abs(min_tilt_angle) - abs(max_tilt_angle) if max_angle < 0 else abs(min_tilt_angle)
        sum = 0.0
        for r in ranges:
            if r[0] < 0:
                delta = abs(r[0] - r[1])/clip_fov if r[1] < 0 else abs(r[0])/clip_fov
                print(f'- "start: {sum}, end: {sum + delta}, downsample: {int(in_rays/(fov/abs(ranges_unclipped[ranges.index(r)][0] - ranges_unclipped[ranges.index(r)][1]))/rays[ranges.index(r)])}"')
                if DEBUG: print(f'[INFO] [range]: {r[0]}, {r[1]}, {rays[ranges.index(r)]}')
                sum += delta

    else:
        print('[ERROR]: with this range the lidar wont see any cone!')

if __name__ == '__main__':
    main()