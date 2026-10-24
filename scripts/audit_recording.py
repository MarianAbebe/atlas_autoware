#!/usr/bin/env python3
"""Read-only bounded payload audit. Source ROS Humble; use /usr/bin/python3.
Scans metadata only for topic counts; deserializes <=3 clouds and <=128 IMUs
per topic at each of three indexed one-second windows. Never rewrites the bag.
"""
import argparse
import json
import sqlite3
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Imu

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('bag')
p.add_argument('--cloud-cdr', help='Optionally export first cloud CDR for contract smoke test')
a = p.parse_args()
c = sqlite3.connect('file:' + a.bag + '?mode=ro', uri=True)
c.execute('PRAGMA query_only=ON')
c.execute('PRAGMA cache_size=-4096')
lo, hi = c.execute('select min(timestamp),max(timestamp) from messages').fetchone()
result = dict(bag=a.bag, start_ns=lo, end_ns=hi, duration_s=(hi-lo)/1e9,
              schema=c.execute('select * from schema').fetchall(), topics=[], samples=[])
counts = dict(c.execute('select topic_id,count(*) from messages group by topic_id'))
for tid, name, typ in c.execute('select id,name,type from topics'):
    result['topics'].append(dict(id=tid, name=name, type=typ, count=counts[tid],
                                count_per_bag_second=counts[tid]/result['duration_s']))
    if typ not in ('sensor_msgs/msg/PointCloud2', 'sensor_msgs/msg/Imu'):
        continue
    for start in (lo, (lo+hi)//2, hi-1_000_000_000):
        rows = c.execute('select timestamp,data from messages indexed by timestamp_idx '
                         'where timestamp>=? and timestamp<? and topic_id=? '
                         'order by timestamp limit ?',
                         (start, start+1_000_000_000, tid, 1 if typ.endswith('PointCloud2') else 128)).fetchall()
        headers = []
        deltas = []
        for t, blob in rows:
            m = deserialize_message(blob, PointCloud2 if typ.endswith('PointCloud2') else Imu)
            h = m.header.stamp.sec*10**9 + m.header.stamp.nanosec
            headers.append(h)
            deltas.append(h-t)
        if not rows:
            continue
        s = dict(topic=name, window_start_ns=start, sampled=len(rows), frame=m.header.frame_id,
                 first_header_ns=headers[0], last_header_ns=headers[-1],
                 header_minus_storage_ns=[min(deltas), max(deltas)],
                 header_monotonic=all(x<y for x,y in zip(headers, headers[1:])))
        if typ.endswith('PointCloud2'):
            if a.cloud_cdr and start == lo:
                with open(a.cloud_cdr, 'wb') as f:
                    f.write(blob)
            s.update(height=m.height,width=m.width,points=m.height*m.width,point_step=m.point_step,
                     row_step=m.row_step,data_bytes=len(m.data),is_dense=m.is_dense,
                     is_bigendian=m.is_bigendian,
                     fields=[dict(name=f.name,offset=f.offset,datatype=f.datatype,count=f.count) for f in m.fields])
            dtype=np.dtype(dict(names=['x','y','z','intensity','ring','time'],
                formats=['<f4','<f4','<f4','<f4','<u2','<f4'],offsets=[0,4,8,12,16,18],itemsize=22))
            if m.point_step != 22 or [(f.name,f.offset,f.datatype,f.count) for f in m.fields] != [
                ('x',0,7,1),('y',4,7,1),('z',8,7,1),('intensity',12,7,1),('ring',16,4,1),('time',18,7,1)]:
                raise RuntimeError('Unexpected layout; refusing fixed-layout analysis')
            pts=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step)).reshape(-1)
            finite=np.isfinite(pts['x']) & np.isfinite(pts['y']) & np.isfinite(pts['z'])
            s['finite_xyz']=int(finite.sum())
            s['ranges']={k:[float(np.nanmin(pts[k])),float(np.nanmax(pts[k]))] for k in dtype.names}
            s['first_points']=[{k:(float(pt[k]) if np.isfinite(pt[k]) else None) for k in dtype.names} for pt in pts[:3]]
            s['time_decreases_in_storage_order']=int(np.sum(np.diff(pts['time'])<0))
        else:
            s.update(header_rate_hz=(len(headers)-1)*1e9/(headers[-1]-headers[0]),
                     orientation=[m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w],
                     orientation_covariance=list(m.orientation_covariance),
                     angular_velocity=[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z],
                     linear_acceleration=[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z])
        result['samples'].append(s)
print(json.dumps(result,indent=2,allow_nan=False))
