"""Disk index for immutable binary XYZ/intensity/RGB optimized PCD snapshots.

Only offsets and tile bounds are indexed; no second full point cloud in RAM.
"""
import math
import os
import sqlite3
import struct
import tempfile
import numpy as np


class OptimizedTiles:
    def __init__(self, path, cancelled=lambda: False):
        self.source = path if hasattr(path, 'read') else open(path, 'rb')
        header = {}
        for _ in range(32):
            line = self.source.readline(4096).decode('ascii').strip()
            if line and not line.startswith('#'):
                key, *values = line.split()
                header[key] = values
            if line == 'DATA binary':
                break
        if (header.get('FIELDS') != ['x', 'y', 'z', 'intensity', 'rgb'] or
                header.get('SIZE') != ['4'] * 5 or header.get('TYPE') != ['F'] * 5 or
                header.get('COUNT') != ['1'] * 5 or header.get('DATA') != ['binary']):
            self.source.close()
            raise ValueError('unsupported optimized PCD layout')
        self.start = self.source.tell()
        self.count = int(header['POINTS'][0])
        if os.fstat(self.source.fileno()).st_size != self.start + self.count * 20:
            self.source.close()
            raise ValueError('truncated optimized PCD')
        self.directory = tempfile.TemporaryDirectory(prefix='scanforge-tiles-')
        self.db = os.path.join(self.directory.name, 'tiles.db')
        with sqlite3.connect(self.db) as db:
            db.execute('PRAGMA journal_mode=OFF')
            db.execute('CREATE TABLE points (x INTEGER,y INTEGER,z INTEGER,offsets BLOB)')
            for first in range(0, self.count, 8192):
                if cancelled():
                    raise InterruptedError('optimized snapshot superseded')
                data = self.source.read(min(8192, self.count-first)*20)
                xyz = np.frombuffer(data, dtype='<f4').reshape(-1, 5)[:, :3]
                valid = np.flatnonzero(np.isfinite(xyz).all(axis=1))
                keys = np.floor(xyz[valid].astype(np.float64)/5).astype(np.int64)
                order = np.lexsort((keys[:, 2], keys[:, 1], keys[:, 0]))
                keys, valid = keys[order], valid[order]
                boundaries = np.r_[0, np.flatnonzero(np.any(keys[1:] != keys[:-1], axis=1))+1, len(keys)]
                rows = [(int(keys[a, 0]), int(keys[a, 1]), int(keys[a, 2]),
                         (self.start+(first+valid[a:b])*20).astype('<i8').tobytes())
                        for a,b in zip(boundaries[:-1], boundaries[1:]) if a < b]
                db.executemany('INSERT INTO points VALUES (?,?,?,?)', rows)
            if cancelled():
                raise InterruptedError('optimized snapshot superseded')
            db.execute('CREATE INDEX tile ON points(x,y,z)')

    def tile(self, x, y, z, limit):
        points = {}
        with sqlite3.connect(self.db) as db:
            # Stream rows; voxel dedup matches raw map's 1 cm detail sampling.
            offsets = (int(offset) for (block,) in db.execute(
                'SELECT offsets FROM points WHERE x=? AND y=? AND z=?', (x,y,z))
                       for offset in np.frombuffer(block, dtype='<i8'))
            for offset in offsets:
                px,py,pz,intensity,rgb = struct.unpack('<ffffI', os.pread(self.source.fileno(),20,offset))
                key = (math.floor(px/.01),math.floor(py/.01),math.floor(pz/.01))
                if key not in points:
                    strength = int(max(0,min(255,intensity if intensity>1 else intensity*255))) if math.isfinite(intensity) else 0
                    points[key] = (px,py,pz,(rgb>>16)&255,(rgb>>8)&255,rgb&255,strength)
                if len(points) >= limit:
                    break
        origin = (x*5,y*5,z*5)
        payload = bytearray(struct.pack('<4sIf3f',b'LIT3',len(points),.01,*origin))
        for px,py,pz,r,g,b,i in points.values():
            payload.extend(struct.pack('<hhhBBBB',round((px-origin[0])*100),round((py-origin[1])*100),round((pz-origin[2])*100),r,g,b,i))
        return bytes(payload)

    def overview(self, limit):
        values = []
        step = max(1, math.ceil(self.count/limit))
        for index in range(0, self.count, step):
            x,y,z,i,rgb = struct.unpack('<ffffI',os.pread(self.source.fileno(),20,self.start+index*20))
            if all(math.isfinite(v) for v in (x,y,z)):
                values.append((x,y,z,(rgb>>16)&255,(rgb>>8)&255,rgb&255,i))
        if not values:
            raise ValueError('empty optimized archive')
        origin = tuple(min(p[k] for p in values) for k in range(3))
        scale = max(.01,max(max(p[k] for p in values)-origin[k] for k in range(3))/32760)
        data = bytearray(struct.pack('<4sIf3f',b'LIO3',len(values),scale,*origin))
        for x,y,z,r,g,b,i in values:
            strength = int(max(0,min(255,i if i>1 else i*255))) if math.isfinite(i) else 0
            data.extend(struct.pack('<hhhBBBB',*(round((v-origin[k])/scale) for k,v in enumerate((x,y,z))),r,g,b,strength))
        return bytes(data),len(values)

    def __del__(self):
        if hasattr(self, 'source'):
            self.source.close()
        if hasattr(self, 'directory'):
            self.directory.cleanup()

    def close(self):
        self.source.close()
        self.directory.cleanup()
