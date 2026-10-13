"""Pose-versioned local archive blocks; no world-point rewrite or full SQL index."""
import itertools
import json
import math
import os
import struct
import threading
from collections import OrderedDict

import numpy as np


def encode(points, magic, origin=None):
    if origin is None:
        if not len(points):
            raise ValueError('empty optimized overview')
        origin = points[:, :3].min(axis=0)
        scale = max(.01, float(np.max(points[:, :3].max(axis=0)-origin))/32760)
    else:
        origin = np.asarray(origin)
        scale = .01
    packed = np.empty(len(points), dtype=[('xyz','<i2',(3,)),('rgba','u1',(4,))])
    packed['xyz'] = np.rint((points[:,:3]-origin)/scale).astype('<i2')
    rgb = points[:,4].astype(np.uint32)
    packed['rgba'][:,0] = (rgb >> 16)&255
    packed['rgba'][:,1] = (rgb >> 8)&255
    packed['rgba'][:,2] = rgb&255
    packed['rgba'][:,3] = np.clip(np.where(points[:,3]>1,points[:,3],points[:,3]*255),0,255).astype(np.uint8)
    return struct.pack('<4sIf3f', magic, len(points), scale, *origin) + packed.tobytes()


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
        # Elementwise arithmetic avoids spawning BLAS threads per small scan.
        return (xyz[:, 0, None]*transform[:, 0] + xyz[:, 1, None]*transform[:, 1] +
                xyz[:, 2, None]*transform[:, 2] + transform[:, 3])

    def records(self, first, count):
        raw = os.pread(self.source.fileno(), count*20, first*20)
        if len(raw) != count*20:
            raise ValueError('archive prefix changed or truncated')
        floats = np.frombuffer(raw, dtype='<f4').reshape(-1,5)
        rgb = np.frombuffer(raw, dtype='<u4').reshape(-1,5)[:,4]
        return floats, rgb

    def overview(self, limit):
        step = max(1, math.ceil(self.count/limit))
        sampled = []
        for first, count, transform in self.blocks:
            if self.cancelled():
                raise InterruptedError('optimized snapshot superseded')
            offset = (-first) % step
            if offset >= count:
                continue
            # Read only overview samples, not every archived point.
            payload = b''.join(os.pread(self.source.fileno(), 20, i*20)
                               for i in range(first+offset, first+count, step))
            selected = np.frombuffer(payload, dtype='<f4').reshape(-1,5)
            rgb = np.frombuffer(payload, dtype='<u4').reshape(-1,5)[:,4]
            xyz = self.transform(selected[:,:3], transform)
            sampled.append(np.column_stack((xyz, selected[:,3], rgb)))
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
        selected, seen = [], set()
        for index in candidates:
            first, count, transform = self.blocks[index]
            raw, rgb = self.records(first, count)
            xyz = self.transform(raw[:,:3], transform)
            valid = np.flatnonzero(np.isfinite(xyz).all(axis=1) &
                                   ((xyz >= origin) & (xyz < origin+5)).all(axis=1))
            for i in valid:
                key = tuple(np.floor(xyz[i]/.01).astype(np.int64))
                if key in seen:
                    continue
                seen.add(key)
                selected.append((*xyz[i], float(raw[i,3]), int(rgb[i])))
                if len(selected) >= limit:
                    return encode(np.array(selected), b'LIT3', origin)
        return encode(np.array(selected).reshape(-1,5), b'LIT3', origin)

    def close(self):
        if self.source is not None:
            self.source.close()

    def __del__(self):
        self.close()
