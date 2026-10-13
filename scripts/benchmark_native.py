#!/usr/bin/python3
"""Synthetic CPU / warm-file benchmarks, not a phone or network performance test.

Run after sourcing the workspace install; defaults compare against e9a8d78.
"""
import argparse
import array
import importlib.util
import json
from pathlib import Path
import statistics
import struct
import subprocess
import tempfile
import time
from types import SimpleNamespace

import numpy as np
from lio_web.cloud import encode_cloud
from lio_web.optimized_blocks import OptimizedBlocks
from lio_web.optimized_tiles import OptimizedTiles


ROOT = Path(__file__).resolve().parents[1]


def measure(function, repeats):
    function()  # warm-up is excluded
    samples = []
    for _ in range(repeats):
        started = time.perf_counter()
        function()
        samples.append(time.perf_counter()-started)
    return statistics.median(samples)*1000


def baseline_module(path, revision):
    source = subprocess.check_output(['git','show',f'{revision}:{path}'],cwd=ROOT,text=True)
    namespace = {'__name__':'benchmark_baseline'}
    exec(compile(source,path,'exec'),namespace)
    return namespace


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--points',type=int,default=200000)
    parser.add_argument('--repeats',type=int,default=3)
    parser.add_argument('--baseline-ref',default='e9a8d78')
    args = parser.parse_args()
    if args.points<=0 or args.repeats<=0:
        parser.error('points and repeats must be positive')
    spec = importlib.util.spec_from_file_location('reference',ROOT/'test/reference_cloud.py')
    ref = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ref)
    old_blocks = baseline_module('lio_web/optimized_blocks.py',args.baseline_ref)['OptimizedBlocks']
    old_tiles = baseline_module('lio_web/optimized_tiles.py',args.baseline_ref)['OptimizedTiles']
    records = np.empty(args.points,dtype=[('xyz','<f4',(3,)),('i','<f4'),('rgb','<u4')])
    indices = np.arange(args.points)
    records['xyz'] = np.column_stack(((indices%200)*.02,((indices//200)%200)*.02,
                                    (indices//40000)*.02))
    records['i'] = 42
    records['rgb'] = 0x123456
    raw = records.tobytes()
    fields = [SimpleNamespace(name=n,offset=i*4,datatype=7)
              for i,n in enumerate(['x','y','z','intensity','rgb'])]
    msg = SimpleNamespace(data=array.array('B',raw),fields=fields,width=args.points,height=1,
                          point_step=20,row_step=args.points*20,is_bigendian=False)
    result = {'points':args.points,'repeats':args.repeats,'baseline_ref':args.baseline_ref,
              'conditions':'synthetic input; median after warm-up; local warm file cache'}
    def compare(name,old,new):
        before,after = old(),new()
        if before != after:
            raise AssertionError(f'{name}: binary output differs')
        a,b = measure(old,args.repeats),measure(new,args.repeats)
        result[name] = {'python_ms':round(a,3),'cpp_ms':round(b,3),'speedup':round(a/b,2)}
    compare('cloud_encode',lambda:ref.encode_cloud(msg,200000,.01),
            lambda:encode_cloud(msg,200000,.01))
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        archive = root/'colored_points.bin'
        archive.write_bytes(raw)
        manifest = root/'optimized_preview.json'
        blocks = [{'first':i,'count':min(8192,args.points-i),
                   'transform':[1,0,0,0,0,1,0,0,0,0,1,0],
                   'bounds':[0,0,0,4,4,float(records['xyz'][:,2].max())]}
                  for i in range(0,args.points,8192)]
        manifest.write_text(json.dumps({'format':'scanforge-blocks-v1','archive':str(archive),
                                        'points':args.points,'blocks':blocks}))
        old,new = old_blocks(manifest.open('rb')),OptimizedBlocks(manifest.open('rb'))
        try:
            compare('block_overview',lambda:old.overview(200000),lambda:new.overview(200000))
            # _tile avoids the encoded cache, which would mask kernel cost.
            compare('block_tile_uncached',lambda:old._tile(0,0,0,30000),
                    lambda:new._tile(0,0,0,30000))
        finally:
            old.close(); new.close()
        pcd = root/'dense.pcd'
        pcd.write_bytes(('FIELDS x y z intensity rgb\nSIZE 4 4 4 4 4\nTYPE F F F F F\n'
                         f'COUNT 1 1 1 1 1\nPOINTS {args.points}\nDATA binary\n').encode()+raw)
        old,new = old_tiles(pcd),OptimizedTiles(pcd)
        try:
            compare('pcd_overview',lambda:old.overview(200000),lambda:new.overview(200000))
            compare('pcd_tile',lambda:old.tile(0,0,0,30000),lambda:new.tile(0,0,0,30000))
        finally:
            old.close(); new.close()
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
