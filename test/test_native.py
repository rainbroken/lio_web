"""Protocol and archive regression tests for the kernels used by the server."""
import array
import struct
from types import SimpleNamespace

import numpy as np
import pytest

from lio_web import _native
from lio_web.cloud import encode_cloud
from reference_cloud import encode_cloud as reference


def message(data, fields, width, height=1, step=20, row_step=None, big=False):
    return SimpleNamespace(
        data=array.array('B', data), fields=[SimpleNamespace(name=n, offset=o, datatype=t)
                                            for n, o, t in fields],
        width=width, height=height, point_step=step,
        row_step=width*step if row_step is None else row_step, is_bigendian=big)


FIELDS = [('x',0,7), ('y',4,7), ('z',8,7), ('intensity',12,7), ('rgb',16,7)]


@pytest.mark.parametrize('magic', [b'LIO3', b'LID3', b'LIT3'])
@pytest.mark.parametrize('limit', [1, 19, 1000])
def test_protocol_matches_frozen_python(magic, limit):
    rng = np.random.default_rng(47)
    xyz = rng.uniform(-20,20,(1000,3)).astype(np.float32)
    xyz[20:40] = xyz[:20]  # First observation wins voxel duplicates.
    xyz[50,0] = np.nan
    intensity = rng.uniform(-2,300,1000).astype(np.float32)
    intensity[:7] = [np.nan, np.inf, 0, .5, 1, 2.5, 3.5]
    raw = b''.join(struct.pack('<ffffI', *p, i, int(c))
                   for p,i,c in zip(xyz,intensity,rng.integers(0,0xffffff,1000)))
    msg = message(raw, FIELDS,1000)
    assert encode_cloud(msg,limit,.01,magic) == reference(msg,limit,.01,magic)


@pytest.mark.parametrize('datatype,fmt', [(1,'b'),(2,'B'),(3,'h'),(4,'H'),
                                        (5,'i'),(6,'I'),(7,'f'),(8,'d')])
def test_numeric_xyz_and_reflectivity(datatype, fmt):
    size = struct.calcsize(fmt)
    fields = [('x',0,datatype),('y',size,datatype),('z',2*size,datatype),
              ('reflectivity',3*size,2)]
    raw = b''.join(struct.pack('<'+fmt*3+'B',x,2,3,127) for x in range(10))
    msg = message(raw,fields,10,step=3*size+1)
    assert encode_cloud(msg,5,.1) == reference(msg,5,.1)


def test_organized_cloud_row_padding_and_big_endian():
    values = [(1,2,3,42,0x123456),(4,5,6,.5,0xabcdef),
              (7,8,9,1,0x010203),(10,11,12,255,0xfefdfc)]
    def organized(endian):
        records = [struct.pack(endian+'ffffI',*v) for v in values]
        return message(b''.join(records[:2])+b'PADDING!'+b''.join(records[2:])+b'PADDING!',
                       FIELDS,2,height=2,row_step=48,big=endian=='>')
    little = organized('<')
    assert encode_cloud(little,10,.01) == reference(little,10,.01)
    assert encode_cloud(organized('>'),10,.01) == encode_cloud(little,10,.01)


def test_invalid_cloud_is_rejected_and_empty_is_preserved():
    assert encode_cloud(message(b'',FIELDS,0),10,.1) == (b'',0)
    assert encode_cloud(message(b'',[],1),10,.1) == (b'',0)
    with pytest.raises(ValueError,match='truncated'):
        encode_cloud(message(b'\x00'*19,FIELDS,1),10,.1)
    with pytest.raises(ValueError,match='field outside'):
        encode_cloud(message(b'\x00'*20,[('x',19,7),('y',4,7),('z',8,7)],1),10,.1)
    for limit,size in [(0,.1),(10,0),(10,float('nan'))]:
        with pytest.raises(ValueError):
            encode_cloud(message(b'\x00'*20,FIELDS,1),limit,size)


def test_archive_reads_order_rgb_bits_and_truncation(tmp_path):
    path = tmp_path/'archive.bin'
    path.write_bytes(b'HEADER'+b''.join(struct.pack('<ffffI',i,2,3,42,0xff123456)
                                      for i in range(200)))
    with path.open('rb') as f:
        for step in [1,3,32,33,1000]:
            result = _native.read_samples(f.fileno(),6,200,step)
            assert np.array_equal(result[:,0],np.arange(0,200,step))
            assert np.all(result[:,4]==0xff123456)
        offsets = np.array([6+20*i for i in [99,1,2,5,33,33,199]],dtype=np.int64)
        result = _native.read_offsets(f.fileno(),offsets)
        assert result[:,0].tolist() == [99,1,2,5,33,33,199]
        with pytest.raises(ValueError,match='truncated'):
            _native.read_samples(f.fileno(),6,201,1)
        with pytest.raises(ValueError):
            _native.read_samples(f.fileno(),6,200,0)
        with pytest.raises(ValueError,match='out of bounds'):
            _native.read_samples(f.fileno(),2**63-1,1,1)
        with pytest.raises(ValueError):
            _native.read_offsets(f.fileno(),np.array([-1],dtype=np.int64))


def test_tile_transform_bounds_dedup_and_budget():
    builder = _native.TileBuilder([-5,0,0],2)
    transform = np.eye(3,4)
    transform[0,3] = -5
    records = np.array([[0,0,0,42,0x123456], [.001,0,0,0,0xffffff],
                        [5,0,0,42,0], [.02,0,0,.5,0xabcdef], [np.nan,0,0,0,0]])
    assert builder.append(records,transform)
    payload = builder.payload()
    assert struct.unpack_from('<I',payload,4)[0] == 2
    assert struct.unpack_from('<hhhBBBB',payload,24) == (0,0,0,0x12,0x34,0x56,42)
    assert struct.unpack_from('<hhhBBBB',payload,34) == (2,0,0,0xab,0xcd,0xef,127)
    assert builder.append(records,transform)  # Reusing builder cannot exceed budget.
    assert builder.payload() == payload


def test_archive_scale_handles_large_extent():
    points = np.array([[-1000,0,0,42,0x123456],[1000,0,0,42,0xabcdef]])
    payload = _native.encode_points(points,'LIO3')
    assert struct.unpack_from('<f',payload,8)[0] > .01
    assert struct.unpack_from('<h',payload,34)[0] == 32760
