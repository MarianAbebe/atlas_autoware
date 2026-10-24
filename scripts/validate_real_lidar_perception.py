#!/usr/bin/env python3
"""Five bounded recorded-cloud tests: conversion -> optional native-frame RANSAC -> clustering.
Requires sourced ROS and a built real_lidar_perception_probe. Never runs a simulator.
"""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import resource
import sqlite3
import subprocess
import tempfile
import time
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from autoware_perception_msgs.msg import DetectedObjects
from smoke_velodyne_recording import check as check_conversion


def points(cloud):
    assert cloud.height==1 and not cloud.is_bigendian
    assert cloud.row_step==cloud.width*cloud.point_step==len(cloud.data)
    fields={f.name:f for f in cloud.fields}
    assert all(fields[k].datatype==7 and fields[k].count==1 for k in ['x','y','z'])
    dtype=np.dtype(dict(names=['x','y','z'],formats=['<f4']*3,
                        offsets=[fields[k].offset for k in ['x','y','z']],itemsize=cloud.point_step))
    data=np.frombuffer(cloud.data,dtype=dtype)
    xyz=np.column_stack([data[k] for k in ['x','y','z']])
    assert np.isfinite(xyz).all()
    return xyz


def cluster_stats(source,debug,objects):
    assert debug.header==objects.header==source.header
    xyz=points(debug); original=points(source)
    assert [f.name for f in debug.fields]==['x','y','z','rgb']
    rgb=next(f for f in debug.fields if f.name=='rgb')
    assert rgb.datatype==7 and rgb.count==1 and rgb.offset+4<=debug.point_step
    colors=np.ndarray((debug.width,),dtype='<u4',buffer=debug.data,
                      offset=rgb.offset,strides=(debug.point_step,)) & 0xffffff
    # Upstream debug output concatenates clusters, cycling six distinct colors.
    # Adjacent clusters always have different colors; contiguous runs reveal sizes.
    bounds=np.r_[0,np.flatnonzero(colors[1:]!=colors[:-1])+1,len(colors)] if len(colors) else np.array([0])
    assert len(bounds)-1==len(objects.objects)
    available=Counter(map(tuple,original.view('<u4')))
    for key,count in Counter(map(tuple,xyz.view('<u4'))).items():
        assert count<=available[key], 'Cluster contains invented or duplicated XYZ'
    stats=[]
    for i,obj in enumerate(objects.objects):
        group=xyz[bounds[i]:bounds[i+1]].astype('f8')
        assert 10<=len(group)<=1000
        assert len(obj.classification)==1 and obj.classification[0].label==0
        centroid=obj.kinematics.pose_with_covariance.pose.position
        center=np.array([centroid.x,centroid.y,centroid.z])
        assert np.isfinite(center).all()
        assert np.allclose(center,group.mean(axis=0),rtol=0,atol=1e-5)
        stats.append(dict(points=len(group),centroid_m=center.tolist(),
                          minimum_xyz_m=group.min(axis=0).tolist(),maximum_xyz_m=group.max(axis=0).tolist(),
                          extent_xyz_m=np.ptp(group,axis=0).tolist()))
    sizes=[s['points'] for s in stats]
    return dict(cluster_count=len(stats),clustered_points=len(xyz),unclustered_points=len(original)-len(xyz),
                size_range=[min(sizes),max(sizes)] if sizes else [],
                all_xyz_finite=True,all_cluster_xyz_from_input=True,clusters=stats)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag',type=Path)
    p.add_argument('--probe',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--reference',type=Path,default=Path.home()/'roam/references/autoware')
    args=p.parse_args()
    args.output_dir.mkdir(parents=True,exist_ok=True)
    if any(args.output_dir.iterdir()): raise RuntimeError('Use a fresh output directory')
    env=dict(os.environ,ROS_LOCALHOST_ONLY='1',ROS_DOMAIN_ID='180',RCUTILS_COLORIZED_OUTPUT='0')
    conn=sqlite3.connect('file:'+str(args.bag.resolve())+'?mode=ro',uri=True)
    conn.execute('PRAGMA query_only=ON');conn.execute('PRAGMA cache_size=-4096')
    lo,hi=conn.execute('select min(timestamp),max(timestamp) from messages').fetchone()
    tid=conn.execute("select id from topics where name='/velodyne_points'").fetchone()[0]
    results=[]
    for index,start in enumerate([lo,lo+(hi-lo)//4,(lo+hi)//2,lo+3*(hi-lo)//4,hi-1_000_000_000]):
        row=conn.execute('select timestamp,data from messages indexed by timestamp_idx '
            'where timestamp>=? and timestamp<? and topic_id=? order by timestamp limit 1',
            (start,start+1_000_000_000,tid)).fetchone()
        if row is None: raise RuntimeError('Missing recorded sample')
        with tempfile.TemporaryDirectory(prefix='atlas_real_perception_') as tmp:
            tmp=Path(tmp);raw=tmp/'raw.cdr';raw.write_bytes(row[1]);env['ROS_LOG_DIR']=str(tmp/'ros_logs')
            began=time.monotonic()
            with (args.output_dir/f'scan_{index}.log').open('w') as log:
                subprocess.run([str(args.probe.resolve()),str(raw),str(tmp)],env=env,
                               stdout=log,stderr=subprocess.STDOUT,timeout=45,check=True)
            wall_ms=(time.monotonic()-began)*1000
            load=lambda name,typ=PointCloud2: deserialize_message((tmp/(name+'.cdr')).read_bytes(),typ)
            source=deserialize_message(row[1],PointCloud2)
            extended,compact,filtered=map(load,['extended','compact','filtered'])
            converted=check_conversion(source,extended,compact)
            assert filtered.header==compact.header and list(filtered.fields)==list(compact.fields)
            assert filtered.point_step==16 and filtered.is_dense
            points(filtered)
            before=np.frombuffer(compact.data,dtype='V16');after=np.frombuffer(filtered.data,dtype='V16')
            pos=0
            for value in after:
                while pos<len(before) and before[pos]!=value:pos+=1
                assert pos<len(before), 'RANSAC modified/invented a point'
                pos+=1
            modes={}
            for mode,cloud in [('filtered',filtered),('bypass',compact)]:
                modes[mode]=cluster_stats(cloud,load(mode+'_clusters'),load(mode+'_objects',DetectedObjects))
            metrics=json.loads((tmp/'metrics.json').read_text())
            assert metrics['process_peak_rss_kib']>0
            assert all(s['publish_to_receipt_ms']>=0 for s in metrics['stages'])
            result=dict(index=index,storage_timestamp_ns=row[0],source_cdr_sha256=hashlib.sha256(row[1]).hexdigest(),
                        source_cdr_bytes=len(row[1]),conversion=converted,ground_filter=dict(
                            input_points=len(before),output_points=len(after),removed_points=len(before)-len(after),
                            byte_identical_ordered_subset=True,ground_reference='unverified sensor-z criterion'),
                        modes=modes,metrics=metrics,whole_probe_wall_ms=wall_ms)
            results.append(result)
            print(json.dumps(dict(index=index,retained=compact.width,filtered=filtered.width,
                clusters={k:v['cluster_count'] for k,v in modes.items()},peak_rss_kib=metrics['process_peak_rss_kib'])),flush=True)
    assert all(sum(s['modes'][mode]['cluster_count'] for s in results)>0 for mode in ['filtered','bypass'])
    provenance={}
    for name,path in dict(probe=str(args.probe),**results[0]['metrics']['libraries'],
                          **{k+'_parameters':v for k,v in results[0]['metrics']['parameter_files'].items()}).items():
        provenance[name]=dict(path=path,sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest())
    revisions={}
    for name,path in [('autoware',args.reference),('core',args.reference/'src/core/autoware_core'),
                      ('universe',args.reference/'src/universe/autoware_universe')]:
        revisions[name]=dict(revision=subprocess.check_output(['git','-C',str(path),'rev-parse','HEAD'],text=True).strip(),
            status=subprocess.check_output(['git','-C',str(path),'status','--short'],text=True).strip())
    report=dict(provenance=provenance,revisions=revisions,bag=str(args.bag),sample_count=len(results),raw_cdr_bytes_read=sum(s['source_cdr_bytes'] for s in results),
                time_contract='explicit FLOAT32 header-relative seconds',
                clustering_parameters=dict(use_height=False,tolerance=0.7,min_cluster_size=10,max_cluster_size=1000),
                latency_definition='steady-clock publication to matching output receipt; includes middleware/executor; cold per scan',
                driver_peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,samples=results)
    (args.output_dir/'results.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')

if __name__=='__main__':
    main()
