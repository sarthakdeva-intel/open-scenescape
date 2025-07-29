# SPDX-FileCopyrightText: (C) 2024 - 2025 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

import base64
import json
import logging
import math
import os
import struct
import time
from collections import defaultdict
from datetime import datetime
from uuid import getnode as get_mac
from pprint import pprint

import cv2
import ntplib
import numpy as np
import paho.mqtt.client as mqtt
from pytz import timezone

from utils import publisher_utils as utils

ROOT_CA = os.environ.get('ROOT_CA', '/run/secrets/certs/scenescape-ca.pem')
DATETIME_FORMAT = "%Y-%m-%dT%H:%M:%S.%f"
TIMEZONE = "UTC"

def getMACAddress():
  if 'MACADDR' in os.environ:
    return os.environ['MACADDR']

  a = get_mac()
  h = iter(hex(a)[2:].zfill(12))
  return ":".join(i + next(h) for i in h)

class PostDecodeTimestampCapture:
  def __init__(self, ntpServer=None):
    self.log = logging.getLogger('SSCAPE_ADAPTER')
    self.log.setLevel(logging.INFO)
    self.ntpClient = ntplib.NTPClient()
    self.ntpServer = ntpServer
    self.lastTimeSync = None
    self.timeOffset = 0
    self.ts = None
    self.timestamp_for_next_block = None
    self.fps = 5.0
    self.fps_alpha = 0.75 # for weighted average
    self.last_calculated_fps_ts = None
    self.fps_calc_interval = 1 # calculate fps every 1s
    self.frame_cnt = 0

  def processFrame(self, frame):
    now = time.time()
    self.frame_cnt += 1
    if not self.last_calculated_fps_ts:
      self.last_calculated_fps_ts = now
    if (now - self.last_calculated_fps_ts) > self.fps_calc_interval:
      self.fps = self.fps * self.fps_alpha + (1 - self.fps_alpha) * (self.frame_cnt / (now - self.last_calculated_fps_ts))
      self.last_calculated_fps_ts = now
      self.frame_cnt = 0

    if self.ntpServer:
      # if ntpServer is available, check if it is time to recalibrate
      if not self.lastTimeSync or now - self.lastTimeSync > 1000 :
        response = self.ntpClient.request(host=self.ntpServer, port=123)
        self.timeOffset = response.offset
        self.lastTimeSync = now

    now += self.timeOffset
    self.timestamp_for_next_block = now
    frame.add_message(json.dumps({
      'postdecode_timestamp': f"{datetime.fromtimestamp(now, tz=timezone(TIMEZONE)).strftime(DATETIME_FORMAT)[:-3]}Z",
      'timestamp_for_next_block': now,
      'fps': self.fps
    }))
    return True

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
  detectionPolicy(pobj, item, fw, fh)
  pobj['text'] = ''
  for key, value in item.items():
    if key.startswith('classification_layer') and isinstance(value, dict) and 'label' in value:
      pobj['text'] = value['label']
      break
  return

metadatapolicies = {
"detectionPolicy": detectionPolicy,
"reidPolicy": reidPolicy,
"classificationPolicy": classificationPolicy,
"ocrPolicy": ocrPolicy
}

class PostInferenceDataPublish:
  def __init__(self, cameraid, metadatagenpolicy='detectionPolicy', publish_image=False):
    self.cameraid = cameraid

    self.is_publish_image = publish_image
    self.is_publish_calibration_image = False
    self.setupMQTT()
    self.metadatagenpolicy = metadatapolicies[metadatagenpolicy]
    self.frame_level_data = {'id': cameraid, 'debug_mac': getMACAddress()}
    return

  def on_connect(self, client, userdata, flags, rc):
    if rc == 0:
      print(f"Connected to MQTT Broker {self.broker}")
      self.client.subscribe(f"scenescape/cmd/camera/{self.cameraid}")
      print(f"Subscribed to topic: scenescape/cmd/camera/{self.cameraid}")
    else:
      print(f"Failed to connect, return code {rc}")
    return

  def setupMQTT(self):
    self.client = mqtt.Client()
    self.client.on_connect = self.on_connect
    self.broker = "broker.scenescape.intel.com"
    self.client.connect(self.broker, 1883, 120)
    self.client.on_message = self.handleCameraMessage
    if ROOT_CA and os.path.exists(ROOT_CA):
      self.client.tls_set(ca_certs=ROOT_CA)
    self.client.loop_start()
    return

  def handleCameraMessage(self, client, userdata, message):
    msg = str(message.payload.decode("utf-8"))
    if msg == "getimage":
      self.is_publish_image = True
    elif msg == "getcalibrationimage":
      self.is_publish_calibration_image = True
    return

  def annotateObjects(self, img):
    objColors = ((0, 0, 255), (255, 128, 128), (207, 83, 294), (31, 156, 238))
    for otype, objects in self.frame_level_data['objects'].items():
      if otype == "person":
        cindex = 0
        # annotation of pose not supported
        #self.annotateHPE(frame, obj)
      elif otype == "vehicle" or otype == "bicycle" or otype == "car":
        cindex = 1
      else:
        cindex = 2
      for obj in objects:
        topleft_cv = (int(obj['bounding_box_px']['x']), int(obj['bounding_box_px']['y']))
        bottomright_cv = (int(obj['bounding_box_px']['x'] + obj['bounding_box_px']['width']),
                        int(obj['bounding_box_px']['y'] + obj['bounding_box_px']['height']))
        cv2.rectangle(img, topleft_cv, bottomright_cv, objColors[cindex], 4)
        
        # Annotate license plates for cars (nested) - draw both bounding box and text
        if otype == "car" and 'license_plates' in obj:
          for plate in obj['license_plates']:
            # Draw license plate bounding box
            plate_topleft = (int(plate['bounding_box_px']['x']), int(plate['bounding_box_px']['y']))
            plate_bottomright = (int(plate['bounding_box_px']['x'] + plate['bounding_box_px']['width']),
                               int(plate['bounding_box_px']['y'] + plate['bounding_box_px']['height']))
            cv2.rectangle(img, plate_topleft, plate_bottomright, objColors[3], 4)  # Use license plate color
            
            # Draw OCR text
            if 'text' in plate and plate['text']:
              self.annotatePlate(img, plate['bounding_box_px'], plate['text'])
    return

  def annotatePlate(self, frame, bounds, text):
    """Annotate license plate text near the license plate bounding box"""
    # Get an estimate of the size of the text with scale 1
    scale = 1
    lsize = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1*scale, 5*scale)[0]

    # Then adjust the scale so the text is about twice the length of the plate
    # (this makes it more or less readable in the annotation without taking too much space)
    if lsize[0] > 0:
      scale = scale * 2 * bounds['width'] / lsize[0]
    lsize = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1*scale, int(5*scale))[0]

    start_x = int(bounds['x'] - lsize[0])
    bottom_y = int(bounds['y'] + 10 + lsize[1])
    end_x = int(bounds['x'])
    top_y = int(bounds['y'] + 10)
    
    # Check if annotation is within image bounds
    if self.pointsInsideImage(frame, [[start_x, top_y], [end_x, bottom_y]]):
      # Draw text with black outline and white fill (for better visibility)
      cv2.putText(frame, text, (start_x, bottom_y),
                  cv2.FONT_HERSHEY_SIMPLEX, 1 * scale, (0,0,0), int(5 * scale))
      cv2.putText(frame, text, (start_x, bottom_y),
                  cv2.FONT_HERSHEY_SIMPLEX, 1 * scale, (255,255,255), int(2 * scale))
    return

  def pointsInsideImage(self, frame, img_pts):
    """Check if points are within image boundaries"""
    frame_height, frame_width = frame.shape[:2]
    for point in img_pts:
      pt_x = int(point[0])
      pt_y = int(point[1])
      if pt_x < 0 or pt_x > frame_width or pt_y < 0 or pt_y > frame_height:
        return False
    return True

  def annotateFPS(self, img, fpsval):
    # code snippet is taken from annotateFPS method in percebro/videoframe.py
    fpsStr = f'FPS {fpsval:.1f}'
    scale = int((img.shape[0] + 479) / 480)
    cv2.putText(img, fpsStr, (0, 30 * scale), cv2.FONT_HERSHEY_SIMPLEX,
            1 * scale, (0,0,0), 5 * scale)
    cv2.putText(img, fpsStr, (0, 30 * scale), cv2.FONT_HERSHEY_SIMPLEX,
            1 * scale, (255,255,255), 2 * scale)
    return

  def buildImgData(self, imgdatadict, gvaframe, annotate):
    imgdatadict.update({
      'timestamp': self.frame_level_data['timestamp'],
      'id': self.cameraid
    })
    with gvaframe.data() as image:
      if annotate:
        self.annotateObjects(image)
        self.annotateFPS(image, self.frame_level_data['rate'])
      _, jpeg = cv2.imencode(".jpg", image)
    jpeg = base64.b64encode(jpeg).decode('utf-8')
    imgdatadict['image'] = jpeg

    return

  def buildObjData(self, gvadata):
    now = time.time()
    self.frame_level_data.update({
      'timestamp': gvadata['postdecode_timestamp'],
      'debug_timestamp_end': f"{datetime.fromtimestamp(now, tz=timezone(TIMEZONE)).strftime(DATETIME_FORMAT)[:-3]}Z",
      'debug_processing_time': now - float(gvadata['timestamp_for_next_block']),
      'rate': float(gvadata['fps'])
    })
    objects = defaultdict(list)
    if 'objects' in gvadata and len(gvadata['objects']) > 0:
      framewidth, frameheight = gvadata['resolution']['width'], gvadata['resolution']['height']
      for det in gvadata['objects']:
        vaobj = {}
        self.metadatagenpolicy(vaobj, det, framewidth, frameheight)
        otype = vaobj['category']
        vaobj['id'] = len(objects[otype]) + 1
        objects[otype].append(vaobj)
    
    # Apply license plate associations after building initial objects
    self.associateLicensePlates(objects)
    self.frame_level_data['objects'] = objects

  def calculateOverlapScore(self, car_bbox, plate_bbox):
    """Calculate overlap score between car and license plate bounding boxes"""
    try:
      # Extract coordinates
      car_x1, car_y1 = car_bbox['x'], car_bbox['y']
      car_x2, car_y2 = car_x1 + car_bbox['width'], car_y1 + car_bbox['height']
      
      plate_x1, plate_y1 = plate_bbox['x'], plate_bbox['y']
      plate_x2, plate_y2 = plate_x1 + plate_bbox['width'], plate_y1 + plate_bbox['height']
      
      # Check if plate center is within expanded car bounds
      plate_center_x = (plate_x1 + plate_x2) / 2
      plate_center_y = (plate_y1 + plate_y2) / 2
      
      # Expand car bounds by 15% to account for license plates on bumpers
      car_width = car_x2 - car_x1
      car_height = car_y2 - car_y1
      margin_x = car_width * 0.15
      margin_y = car_height * 0.15
      
      expanded_car_x1 = car_x1 - margin_x
      expanded_car_y1 = car_y1 - margin_y
      expanded_car_x2 = car_x2 + margin_x
      expanded_car_y2 = car_y2 + margin_y
      
      # Check if plate center is within expanded car
      if (expanded_car_x1 <= plate_center_x <= expanded_car_x2 and
          expanded_car_y1 <= plate_center_y <= expanded_car_y2):
        # Calculate distance-based score (closer = higher score)
        car_center_x = (car_x1 + car_x2) / 2
        car_center_y = (car_y1 + car_y2) / 2
        distance = ((plate_center_x - car_center_x) ** 2 + (plate_center_y - car_center_y) ** 2) ** 0.5
        # Normalize distance by car diagonal
        car_diagonal = (car_width ** 2 + car_height ** 2) ** 0.5
        if car_diagonal > 0:
          normalized_distance = distance / car_diagonal
          return max(0, 1.0 - normalized_distance)
      
      return 0.0
    except Exception as e:
      print(f"Error calculating overlap score: {e}")
      return 0.0

  def associateLicensePlates(self, objects):
    """Create associations between cars and license plates"""
    cars = objects.get('car', [])
    license_plates = objects.get('license_plate', [])
    
    if not cars or not license_plates:
      return
    
    print(f"Associating {len(cars)} cars with {len(license_plates)} license plates")
    
    used_plates = set()
    associations_created = 0
    
    for car_idx, car in enumerate(cars):
      car_bbox = car['bounding_box_px']
      associated_plates = []
      
      # Find best matching license plates
      plate_scores = []
      for plate_idx, plate in enumerate(license_plates):
        if plate_idx in used_plates:
          continue
          
        plate_bbox = plate['bounding_box_px']
        score = self.calculateOverlapScore(car_bbox, plate_bbox)
        if score > 0.1:  # Minimum threshold
          plate_scores.append((score, plate_idx, plate))
      
      # Sort by score and take the best matches
      plate_scores.sort(reverse=True, key=lambda x: x[0])
      
      for score, plate_idx, plate in plate_scores[:2]:  # Max 2 plates per car
        # Extract OCR text
        plate_info = {
          'bounding_box_px': plate['bounding_box_px'],
          'confidence': plate['confidence'],
          'text': plate['text']
        }
        associated_plates.append(plate_info)
        used_plates.add(plate_idx)
        associations_created += 1
        
        print(f"Associated car {car_idx} with license plate {plate_idx} (score: {score:.2f}, text: '{plate['text']}')")
      
      # Add license plates to car object
      if associated_plates:
        car['license_plates'] = associated_plates
    
    # Remove associated license plates from the main objects list
    if used_plates:
      remaining_plates = [plate for idx, plate in enumerate(license_plates) if idx not in used_plates]
      if remaining_plates:
        objects['license_plate'] = remaining_plates
        print(f"Kept {len(remaining_plates)} unassociated license plates in main objects list")
      else:
        # Remove license_plate category entirely if all plates were associated
        if 'license_plate' in objects:
          del objects['license_plate']
        print("All license plates were associated with cars, removed license_plate category from main objects list")
    
    print(f"Created {associations_created} license plate associations")

  def processFrame(self, frame):
    if self.client.is_connected():
      gvametadata, imgdatadict = {}, {}

      utils.get_gva_meta_messages(frame, gvametadata)
      gvametadata['gva_meta'] = utils.get_gva_meta_regions(frame)

      self.buildObjData(gvametadata)

      if self.is_publish_image:
        self.buildImgData(imgdatadict, frame, True)
        self.client.publish(f"scenescape/image/camera/{self.cameraid}", json.dumps(imgdatadict))
        self.is_publish_image = False

      if self.is_publish_calibration_image:
        if not imgdatadict:
          self.buildImgData(imgdatadict, frame, False)
        self.client.publish(f"scenescape/image/calibration/camera/{self.cameraid}", json.dumps(imgdatadict))
        self.is_publish_calibration_image = False

      self.client.publish(f"scenescape/data/camera/{self.cameraid}", json.dumps(self.frame_level_data))
      frame.add_message(json.dumps(self.frame_level_data))
    return True
