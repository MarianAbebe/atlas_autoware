#!/usr/bin/env python3
"""Bounded real-cloud DDS smoke test; run after sourcing the new Atlas install.
No TF, motion, /clock, or sensor geometry is fabricated. Reads three clouds only.
"""
import argparse
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import tempfile
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2


def check(source, extended, compact):
    fields = {f.name: f for f in source.fields}
    names = ['x', 'y', 'z', 'intensity', 'ring', 'time']
    endian = '>' if source.is_bigendian else '<'
    dt = np.dtype(dict(names=names, formats=[endian+'f4']*4+[endian+'u2',endian+'f4'],
                       offsets=[fields[n].offset for n in names], itemsize=source.point_step))
    points = np.ndarray((source.height,source.width), dtype=dt, buffer=source.data,
                        strides=(source.row_step, source.point_step)).reshape(-1)
    good = np.isfinite(points['x']) & np.isfinite(points['y']) & np.isfinite(points['z'])
    kept = points[good]
    order = np.argsort(kept['time'], kind='stable')
    kept = kept[order]
    assert extended.header == compact.header == source.header
    assert extended.height == compact.height == 1
    assert extended.width == compact.width == len(kept)
    assert extended.point_step == 32 and compact.point_step == 16
    assert extended.row_step == extended.width*32 and compact.row_step == compact.width*16
    assert extended.is_dense and compact.is_dense
    assert not extended.is_bigendian and not compact.is_bigendian
    out = np.frombuffer(extended.data, dtype=np.dtype(dict(
        names=['x','y','z','intensity','return_type','channel','azimuth','elevation','distance','time_stamp'],
        formats=['<f4','<f4','<f4','u1','u1','<u2','<f4','<f4','<f4','<u4'],
        offsets=[0,4,8,12,13,14,16,20,24,28], itemsize=32)))
    assert [(f.name,f.offset,f.datatype,f.count) for f in extended.fields] == [
        ('x',0,7,1),('y',4,7,1),('z',8,7,1),('intensity',12,2,1),('return_type',13,2,1),
        ('channel',14,4,1),('azimuth',16,7,1),('elevation',20,7,1),('distance',24,7,1),('time_stamp',28,6,1)]
    assert list(compact.fields) == list(extended.fields[:6])
    for axis in ['x','y','z']:
        # Bitwise equality, not a tolerance: every finite source point survives.
        assert np.array_equal(out[axis].view('<u4'),kept[axis].view('<u4'))
    assert np.array_equal(out['channel'],kept['ring'])
    assert np.all(out['return_type'] == 0)
    intensity = np.where(np.isfinite(kept['intensity']),kept['intensity'],0)
    assert np.array_equal(out['intensity'],np.floor(np.clip(intensity,0,255)+0.5).astype('u1'))
    expected_ns = np.floor(kept['time'].astype('f8')*1e9+0.5).astype('<u4')
    assert np.array_equal(out['time_stamp'],expected_ns)
    assert np.all(out['time_stamp'][1:] >= out['time_stamp'][:-1])
    xyz = np.column_stack([out[k] for k in ['x','y','z']]).astype('f8')
    assert np.isfinite(xyz).all()
    assert np.allclose(out['distance'],np.linalg.norm(xyz,axis=1),rtol=2e-7,atol=1e-6)
    assert np.isfinite(out['azimuth']).all() and np.isfinite(out['elevation']).all()
    assert np.array_equal(np.frombuffer(compact.data,dtype='u1').reshape(-1,16),
                          np.frombuffer(extended.data,dtype='u1').reshape(-1,32)[:,:16])
    return dict(stamp_sec=source.header.stamp.sec, stamp_nanosec=source.header.stamp.nanosec,
                frame=source.header.frame_id,input_points=len(points),output_points=len(kept),
                discarded_nonfinite_xyz=int((~good).sum()),reordered=bool(np.any(order!=np.arange(len(order)))),
                time_ns_range=[int(out['time_stamp'].min()),int(out['time_stamp'].max())],
                ring_values=np.unique(out['channel']).tolist(),xyz_bitwise_preserved=True,
                compact_bytes_preserved=True,maximum_time_rounding_error_ns=float(np.max(np.abs(
                    out['time_stamp'].astype('f8')-kept['time'].astype('f8')*1e9))))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('--probe',required=True,help='Built real_cloud_smoke executable')
    parser.add_argument('--output',required=True)
    parser.add_argument('--deskew-probe',help='Optional pinned no-motion boundary executable')
    parser.add_argument('--ground-probe',help='Optional pinned native sensor-frame RANSAC executable')
    args=parser.parse_args()
    env=os.environ.copy()
    env.update(ROS_LOCALHOST_ONLY='1',ROS_DOMAIN_ID='179')
    conn=sqlite3.connect('file:'+args.bag+'?mode=ro',uri=True)
    conn.execute('PRAGMA query_only=ON'); conn.execute('PRAGMA cache_size=-4096')
    lo,hi=conn.execute('select min(timestamp),max(timestamp) from messages').fetchone()
    tid=conn.execute("select id from topics where name='/velodyne_points'").fetchone()[0]
    results=[]
    with tempfile.TemporaryDirectory(prefix='atlas_velodyne_smoke_') as tmp:
        env['ROS_LOG_DIR']=tmp
        for start in [lo,(lo+hi)//2,hi-1_000_000_000]:
            row=conn.execute('select data from messages indexed by timestamp_idx '
                'where timestamp>=? and timestamp<? and topic_id=? order by timestamp limit 1',
                (start,start+1_000_000_000,tid)).fetchone()
            if row is None: raise RuntimeError('No cloud in requested sample window')
            paths=[Path(tmp)/name for name in ['input.cdr','extended.cdr','compact.cdr']]
            paths[0].write_bytes(row[0])
            subprocess.run([args.probe,*map(str,paths)],env=env,check=True,timeout=20)
            clouds=[deserialize_message(p.read_bytes(),PointCloud2) for p in paths]
            result=check(*clouds)
            if args.deskew_probe:
                subprocess.run([args.deskew_probe,str(paths[1])],env=env,check=True,timeout=20)
                result['deskew_boundary']='pinned layout accepted; absent twist; unchanged; no compensation'
            if args.ground_probe:
                ground_path=Path(tmp)/'ground_filtered.cdr'
                subprocess.run([args.ground_probe,str(paths[2]),str(ground_path)],env=env,check=True,timeout=25)
                filtered=deserialize_message(ground_path.read_bytes(),PointCloud2)
                compact=clouds[2]
                assert filtered.header==compact.header and list(filtered.fields)==list(compact.fields)
                assert filtered.point_step==compact.point_step and filtered.height==1
                assert filtered.row_step==filtered.width*filtered.point_step==len(filtered.data)
                assert not filtered.is_bigendian and filtered.is_dense
                # Verify an ordered byte-for-byte subset, including ring and intensity.
                original=np.frombuffer(compact.data,dtype='V16')
                remaining=np.frombuffer(filtered.data,dtype='V16')
                pos=0
                for value in remaining:
                    while pos<len(original) and original[pos]!=value:
                        pos+=1
                    assert pos<len(original), 'Ground filter invented/changed a point'
                    pos+=1
                assert len(remaining)<=len(original)
                result['ransac_sensor_frame']=dict(input_points=len(original),output_points=len(remaining),
                    removed_points=len(original)-len(remaining),ordered_byte_subset=True,
                    ground_semantics_verified=False)
            results.append(result)
    Path(args.output).write_text(json.dumps(dict(bag=args.bag,
        transport='registered ROS components, single process, intra-process communication disabled',
        time_contract='explicit header-relative FLOAT32 seconds',samples=results),indent=2)+'\n')
    print(json.dumps(results,indent=2))

if __name__=='__main__':
    main()
