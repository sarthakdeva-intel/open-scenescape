# SPDX-FileCopyrightText: (C) 2024 - 2025 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

import math
from scipy.spatial.transform import Rotation
import numpy as np
import cv2
import struct
import base64
import os
from uuid import getnode as get_mac

## Polices to post process data from the detector

def detectionPolicy(pobj, item, fw, fh):
  pobj.update({
    'category': item['detection']['label'],
    'confidence': item['detection']['confidence']
  })
  computeObjBoundingBoxParams(pobj, fw, fh, item['x'], item['y'], item['w'],item['h'],
                              item['detection']['bounding_box']['x_min'],
                              item['detection']['bounding_box']['y_min'],
                              item['detection']['bounding_box']['x_max'],
                              item['detection']['bounding_box']['y_max'])
  return

def detection3DPolicy(pobj, item, fw, fh):
  pobj.update({
    'category': item['detection']['label'],
    'confidence': item['detection']['confidence'],
  })

  if 'extra_params' in item:
    pobj.update({
      'translation': item['extra_params']['translation'],
      'rotation': item['extra_params']['rotation'],
      'size': item['extra_params']['dimension']
    })
  
    x_min, y_min, z_min = pobj['translation']
    x_size, y_size, z_size = pobj['size']
    x_max, y_max, z_max = x_min + x_size, y_min + y_size, z_min + z_size
    
    bbox_width = x_max - x_min
    bbox_height = y_max - y_min
    bbox_depth = z_max - z_min
    
    com_w, com_h, com_d = bbox_width / 3, bbox_height / 4, bbox_depth / 3
    
    com_x = int(x_min + com_w)
    com_y = int(y_min + com_h) 
    com_z = int(z_min + com_d)

    pobj['bounding_box_3D'] = {
      'x': x_min,
      'y': y_min,
      'z': z_min,
      'width': bbox_width,
      'height': bbox_height,
      'depth': bbox_depth
    }
    pobj['center_of_mass'] = {
      'x': com_x,
      'y': com_y, 
      'z': com_z,
      'width': com_w,
      'height': com_h,
      'depth': com_d
    }
  else:
    computeObjBoundingBoxParams(pobj, fw, fh, item['x'], item['y'], item['w'],item['h'],
                            item['detection']['bounding_box']['x_min'],
                            item['detection']['bounding_box']['y_min'],
                            item['detection']['bounding_box']['x_max'],
                            item['detection']['bounding_box']['y_max'])
  if 'bounding_box_px' in pobj or 'rotation' in pobj:
    pass
  else:
    print(f"Warning: No bounding box or rotation data found in item {item}")
  return

def reidPolicy(pobj, item, fw, fh):
  detectionPolicy(pobj, item, fw, fh)
  reid_vector = item['tensors'][1]['data']
  # following code snippet is from percebro/modelchain.py
  v = struct.pack("256f",*reid_vector)
  pobj['reid'] = base64.b64encode(v).decode('utf-8')
  return

def classificationPolicy(pobj, item, fw, fh):
  detectionPolicy(pobj, item, fw, fh)
  # todo: add configurable parameters(set tensor name)
  pobj['category'] = item['classification_layer_name:efficientnet-b0/model/head/dense/BiasAdd:0']['label']
  return

def ocrPolicy(pobj, item, fw, fh):
  """Extract OCR text from classification layers"""
  detection3DPolicy(pobj, item, fw, fh)
  pobj['text'] = ''
  for key, value in item.items():
    if key.startswith('classification_layer') and isinstance(value, dict) and 'label' in value:
      pobj['text'] = value['label']
      break
  return

## Utility functions

def getMACAddress():
  if 'MACADDR' in os.environ:
    return os.environ['MACADDR']

  a = get_mac()
  h = iter(hex(a)[2:].zfill(12))
  return ":".join(i + next(h) for i in h)

def computeObjBoundingBoxParams(pobj, fw, fh, x, y, w, h, xminnorm=None, yminnorm=None, xmaxnorm=None, ymaxnorm=None):
  # use normalized bounding box for calculating center of mass
  xmax, xmin = int(xmaxnorm * fw), int(xminnorm * fw)
  ymax, ymin = int(ymaxnorm * fh), int(yminnorm * fh)
  comw, comh = (xmax - xmin) / 3, (ymax - ymin) / 4

  pobj.update({
    'center_of_mass': {'x': int(xmin + comw), 'y': int(ymin + comh), 'width': comw, 'height': comh},
    'bounding_box_px': {'x': x, 'y': y, 'width': w, 'height': h}
  })
  return

def getCuboidVertices(bbox3D, rotation=None):
  """ Creates vertices for cuboid based on (x, y, z) and (width, height, depth)."""

  width = bbox3D['width']
  height = bbox3D['height']
  depth = bbox3D['depth']
  x = bbox3D['x']
  y = bbox3D['y']
  z = bbox3D['z']

  vertices = np.zeros([3, 8])

  # Setup X, Y and Z respectively
  vertices[0, [0, 1, 4, 5]], vertices[0, [2, 3, 6, 7]] = width / 2, -width / 2
  vertices[1, [0, 3, 4, 7]], vertices[1, [1, 2, 5, 6]] = height / 2, -height / 2
  vertices[2, [0, 1, 2, 3]], vertices[2, [4, 5, 6, 7]] = 0, depth

  # Rotate
  if rotation is not None:
    if len(rotation) == 3:
      vertices = rotationAsMatrix(rotation) @ vertices
    elif len(rotation) == 4:
      vertices = Rotation.from_quat(rotation).as_matrix() @ vertices

  # Translate
  vertices[0, :] += x
  vertices[1, :] += y
  vertices[2, :] += z

  vertices = np.transpose(vertices)
  return vertices

def rotationAsMatrix(rotation):
  rotation_x = np.array([
    [1, 0, 0],
    [0, math.cos(rotation[0]), -math.sin(rotation[0])],
    [0, math.sin(rotation[0]), math.cos(rotation[0])]
  ])

  rotation_y = np.array([
    [math.cos(rotation[1]), 0, math.sin(rotation[1])],
    [0, 1, 0],
    [-math.sin(rotation[1]), 0, math.cos(rotation[1])]
  ])

  rotation_z = np.array([
    [math.cos(rotation[2]), -math.sin(rotation[2]), 0],
    [math.sin(rotation[2]), math.cos(rotation[2]), 0],
    [0, 0, 1]
  ])

  rotation_as_matrix = np.dot(rotation_z, np.dot(rotation_y, rotation_x))
  return rotation_as_matrix

def annotate3DObject(img, obj, intrinsics, color=(66, 186, 150), thickness=2):
    """Annotate 3D object on the image"""
    vertex_idxs = [0, 1, 2, 3, 7, 6, 5, 4, 7, 3, 0, 4, 5, 1, 2, 6]
    rotation = obj['rotation']
    
    # Create cuboid vertices based on translation, rotation, and size
    vertices = getCuboidVertices(obj['bounding_box_3D'], rotation)
    intrinsics = np.array(intrinsics).reshape(3, 3)
    pts_img = intrinsics @ vertices.T
    transformed_vertices = None
    if np.all(np.absolute(pts_img[2]) > 1e-7) :
      pts_img = pts_img[:2] / pts_img[2]
      transformed_vertices = pts_img.T.astype(np.int32)
    else:
      print("Division by zero: bbox", obj['bounding_box_3D'],
              "image coords", pts_img)
      return
    
    if transformed_vertices is None:
      return
    
    for idx in range(len(vertex_idxs)-1):
      cv2.line( img,
                transformed_vertices[vertex_idxs[idx]],
                transformed_vertices[vertex_idxs[idx+1]],
                color=(255,0,0) if idx == 0 else color,
                thickness=2 )
    return