"""Pose-versioned local archive blocks; no world-point rewrite or full SQL index."""
import itertools
import json
import math
import os
import threading
from collections import OrderedDict

import numpy as np
from lio_web import _native


def encode(points, magic, origin=None):
    return _native.encode_points(points, magic.decode('ascii'), origin)


class OptimizedBlocks:
    def __init__(self, manifest, cancelled=lambda: False):
        self.source = None
        with manifest:
            document = json.load(manifest)
        if document.get('format') != 'scanforge-blocks-v1':
            raise ValueError('unsupported block manifest')
        self.count = int(document['points'])
        if self.count <= 0:
            raise ValueError('empty block archive')
        self.source = open(document['archive'], 'rb')
        if os.fstat(self.source.fileno()).st_size < self.count*20:
            raise ValueError('truncated archive prefix')
        self.blocks = []
        minimum, maximum = [], []
        first = 0
        for block in document['blocks']:
            if cancelled():
                raise InterruptedError('optimized snapshot superseded')
            count = int(block['count'])
            transform = np.asarray(block['transform'], dtype=np.float64).reshape(3,4)
            bounds = np.asarray(block['bounds'], dtype=np.float64).reshape(2,3)
            if (int(block['first']) != first or count <= 0 or first+count > self.count or
                    not np.isfinite(transform).all() or not np.isfinite(bounds).all() or
                    np.any(bounds[0] > bounds[1])):
                raise ValueError('invalid block prefix or transform')
            corners = np.array(list(itertools.product(*zip(bounds[0], bounds[1]))))
            corners = self.transform(corners, transform)
            minimum.append(corners.min(axis=0)); maximum.append(corners.max(axis=0))
            self.blocks.append((first, count, transform))
            first += count
        if first != self.count:
            raise ValueError('incomplete block coverage')
        self.minimum, self.maximum = np.array(minimum), np.array(maximum)
        self.cancelled = cancelled
        self.cache = OrderedDict()
        self.cache_bytes = 0
        self.cache_lock = threading.Lock()

    @staticmethod
    def transform(xyz, transform):
        return _native.transform(xyz, transform)

    def records(self, first, count):
        return _native.read_samples(self.source.fileno(), first*20, count, 1)

    def overview(self, limit):
        if limit <= 0:
            raise ValueError('overview limit must be positive')
        step = max(1, math.ceil(self.count/limit))
        sampled = []
        for first, count, transform in self.blocks:
            if self.cancelled():
                raise InterruptedError('optimized snapshot superseded')
            offset = (-first) % step
            if offset >= count:
                continue
            # Bounded chunks retain cancellation points for very large blocks.
            for begin in range(first+offset, first+count, step*8192):
                if self.cancelled():
                    raise InterruptedError('optimized snapshot superseded')
                selected = _native.read_samples(
                    self.source.fileno(), begin*20,
                    min(step*8192, first+count-begin), step)
                selected[:, :3] = self.transform(selected[:, :3], transform)
                sampled.append(selected)
        points = np.concatenate(sampled) if sampled else np.empty((0,5))
        points = points[np.isfinite(points).all(axis=1)]
        return encode(points, b'LIO3'), len(points)

    def tile(self, x, y, z, limit):
        key = (x,y,z,limit)
        with self.cache_lock:
            if key in self.cache:
                self.cache.move_to_end(key)
                return self.cache[key]
        payload = self._tile(x,y,z,limit)
        with self.cache_lock:
            if key not in self.cache:
                self.cache[key] = payload
                self.cache_bytes += len(payload)
                while self.cache_bytes > 16*1024*1024:
                    _, old = self.cache.popitem(last=False)
                    self.cache_bytes -= len(old)
        return payload

    def _tile(self, x, y, z, limit):
        origin = np.array([x,y,z], dtype=np.float64)*5
        candidates = np.flatnonzero(((self.maximum >= origin) & (self.minimum < origin+5)).all(axis=1))
        # Interleave observations in time so a bounded tile covers the scan.
        if len(candidates):
            stride = max(1, len(candidates)//32)
            candidates = np.concatenate([candidates[i::stride] for i in range(stride)])
        builder = _native.TileBuilder(origin, limit)
        for index in candidates:
            first, count, transform = self.blocks[index]
            for begin in range(first, first+count, 8192):
                raw = self.records(begin, min(8192, first+count-begin))
                if builder.append(raw, transform):
                    return builder.payload()
        return builder.payload()

    def close(self):
        if self.source is not None:
            self.source.close()

    def __del__(self):
        self.close()
