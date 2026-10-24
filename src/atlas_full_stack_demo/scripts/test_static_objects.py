#!/usr/bin/env python3
import copy,math,unittest
import numpy as np
from geometry_msgs.msg import Transform
from sensor_msgs.msg import PointCloud2,PointField
from autoware_perception_msgs.msg import DetectedObjects,DetectedObject
from static_objects import measured_bounds,predictions

class Contracts(unittest.TestCase):
    def inputs(self):
        msg=DetectedObjects();msg.header.frame_id='synthetic_lidar';msg.header.stamp.sec=123
        d=DetectedObject();d.kinematics.pose_with_covariance.pose.position.x=2.;d.kinematics.pose_with_covariance.pose.position.z=.5;msg.objects=[d]
        xyz=np.array([[1.6,-.3,.1],[2.4,.3,.9]],dtype='<f4');a=np.zeros(2,dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')])
        for i,k in enumerate(['x','y','z']):a[k]=xyz[:,i]
        a['rgb']=0xff0000
        cloud=PointCloud2();cloud.header=copy.deepcopy(msg.header);cloud.width=2;cloud.height=1;cloud.point_step=16;cloud.row_step=32;cloud.data=a.tobytes();cloud.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(['x','y','z','rgb'])]
        return msg,cloud
    def test_measured_bounds_transform_and_static_path(self):
        m,c=self.inputs();bounds=measured_bounds(m,c);tf=Transform();tf.translation.x=10.;tf.translation.y=20.;tf.rotation.z=math.sin(math.pi/4);tf.rotation.w=math.cos(math.pi/4)
        out=predictions(m,tf,bounds);p=out.objects[0];pose=p.kinematics.initial_pose_with_covariance.pose
        self.assertEqual(out.header.stamp,m.header.stamp);self.assertEqual(out.header.frame_id,'map');self.assertAlmostEqual(pose.position.x,10.);self.assertAlmostEqual(pose.position.y,22.)
        self.assertAlmostEqual(p.shape.dimensions.x,.8,places=5);self.assertTrue(all(q==pose for q in p.kinematics.predicted_paths[0].path));self.assertEqual(len(p.kinematics.predicted_paths[0].path),11)
    def test_empty_detections_stay_empty(self):
        m=DetectedObjects();c=PointCloud2();self.assertEqual(measured_bounds(m,c),[]);self.assertEqual(predictions(m,Transform(),[]).objects,[])
    def test_centroid_mismatch_rejected(self):
        m,c=self.inputs();m.objects[0].kinematics.pose_with_covariance.pose.position.x=20.
        with self.assertRaises(ValueError):measured_bounds(m,c)
    def test_header_mismatch_rejected(self):
        m,c=self.inputs();c.header.frame_id='map'
        with self.assertRaises(ValueError):measured_bounds(m,c)
if __name__=='__main__':unittest.main()
