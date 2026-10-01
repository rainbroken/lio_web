"""Bounded Web trajectory with detailed recent poses and simplified history."""

import heapq
import math


def _distance_to_segment_squared(point, start, end):
    direction = [end[i] - start[i] for i in range(3)]
    length_squared = sum(value * value for value in direction)
    if length_squared == 0.0:
        return sum((point[i] - start[i]) ** 2 for i in range(3))
    fraction = max(0.0, min(1.0, sum(
        (point[i] - start[i]) * direction[i] for i in range(3)) / length_squared))
    return sum((point[i] - start[i] - fraction * direction[i]) ** 2
               for i in range(3))


def _simplify_old(points, limit):
    """Remove low-curvature interior poses while retaining both endpoints."""
    if len(points) <= limit:
        return points
    count = len(points)
    previous = [i - 1 for i in range(count)]
    following = [i + 1 for i in range(count)]
    removed = [False] * count
    versions = [0] * count
    heap = []

    def enqueue(index):
        if index <= 0 or index >= count - 1 or removed[index]:
            return
        versions[index] += 1
        error = _distance_to_segment_squared(
            points[index], points[previous[index]], points[following[index]])
        heapq.heappush(heap, (error, index, versions[index]))

    for index in range(1, count - 1):
        enqueue(index)
    remaining = count
    while remaining > limit:
        _, index, version = heapq.heappop(heap)
        if removed[index] or version != versions[index]:
            continue
        before, after = previous[index], following[index]
        removed[index] = True
        following[before] = after
        previous[after] = before
        remaining -= 1
        enqueue(before)
        enqueue(after)

    result = []
    index = 0
    while index < count:
        result.append(points[index])
        index = following[index]
    return result


class TrajectoryHistory:
    def __init__(self, max_points, min_step=0.05):
        if max_points < 4:
            raise ValueError('trajectory_max_points must be at least 4')
        self.max_points = max_points
        self.min_step = min_step
        self._points = []

    def clear(self):
        self._points.clear()

    def append(self, pose):
        if self._points and math.dist(pose, self._points[-1]) < self.min_step:
            return
        self._points.append(pose)
        if len(self._points) <= self.max_points:
            return
        recent_count = max(2, self.max_points // 4)
        old = self._points[:-recent_count]
        old_limit = max(2, len(old) * 3 // 4)
        self._points = _simplify_old(old, old_limit) + self._points[-recent_count:]

    def snapshot(self):
        return list(self._points)
