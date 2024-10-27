from mmdet3d.apis import init_model, inference_detector

config_file = 'pointpillars_hv_secfpn_8xb6-160e_kitti-3d-car.py'
checkpoint_file = 'hv_pointpillars_secfpn_6x8_160e_kitti-3d-car_20220331_134606-d42d15ed.pth'
model = init_model(config_file, checkpoint_file)

# Run inference on point cloud
result = inference_detector(model, 'demo/data/kitti/000008.bin')


# Result structure
pred_instances = result[0].pred_instances_3d
boxes = pred_instances.bboxes_3d      # 3D bounding boxes (N, 7) - x,y,z,l,w,h,yaw
scores = pred_instances.scores_3d     # Confidence scores (N,)
labels = pred_instances.labels_3d     # Class labels (N,)

# print(pred_instances)
print(boxes)
print(scores)
print(labels)