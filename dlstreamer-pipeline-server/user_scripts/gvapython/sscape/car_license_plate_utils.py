# SPDX-FileCopyrightText: (C) 2024 - 2025 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

VEHICLE_BOUNDS_BUFFER = 0.15


class CarLicensePlateProcessor:
    """Utility class for processing car and license plate associations and annotations using 3D bounding boxes"""
    
    def __init__(self):
        self.associations_created = 0
        
    def getCuboidVertices(self, bbox3D, rotation=None):
        """Creates vertices for cuboid based on (x, y, z) and (width, height, depth)"""
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
        
        # Rotate if rotation is provided
        if rotation is not None:
            if len(rotation) == 4:  # quaternion
                vertices = Rotation.from_quat(rotation).as_matrix() @ vertices
        
        # Translate
        vertices[0, :] += x
        vertices[1, :] += y  
        vertices[2, :] += z
        
        vertices = np.transpose(vertices)
        return vertices
    
    def findClosestFace(self, vertices):
        """Find the closest face of the 3D bounding box (smallest average z)"""
        # Define the 6 faces by their 4 corner indices
        faces = [
            [0, 1, 2, 3],  # bottom
            [4, 5, 6, 7],  # top  
            [0, 1, 5, 4],  # front
            [2, 3, 7, 6],  # back
            [1, 2, 6, 5],  # right
            [0, 3, 7, 4]   # left
        ]
        
        min_face = 0
        min_z = float('inf')
        
        for f, face in enumerate(faces):
            z_avg = np.mean([vertices[i][2] for i in face])
            if z_avg < min_z:
                min_z = z_avg
                min_face = f
                
        return faces[min_face]
    
    def project3DTo2D(self, vertices, intrinsics):
        """Project 3D vertices to 2D using camera intrinsics"""
        if intrinsics is None:
            return None
            
        intrinsics = np.array(intrinsics).reshape(3, 3)
        pts_img = intrinsics @ vertices.T
        
        if np.all(np.absolute(pts_img[2]) > 1e-7):
            pts_img = pts_img[:2] / pts_img[2]
            return pts_img.T.astype(np.int32)
        else:
            return None
    
    def calculate3DBounds2D(self, car_3d, intrinsics):
        """Calculate 2D projection bounds of a 3D car bounding box"""
        try:
            # Get 3D vertices
            vertices = self.getCuboidVertices(car_3d['bounding_box_3D'], car_3d.get('rotation'))
            
            # Project to 2D
            projected_vertices = self.project3DTo2D(vertices, intrinsics)
            if projected_vertices is None:
                return None
                
            # Get bounding rectangle
            x_coords = projected_vertices[:, 0]
            y_coords = projected_vertices[:, 1] 
            
            return {
                'x': int(np.min(x_coords)),
                'y': int(np.min(y_coords)),
                'width': int(np.max(x_coords) - np.min(x_coords)),
                'height': int(np.max(y_coords) - np.min(y_coords))
            }
        except Exception as e:
            print(f"Error calculating 3D bounds: {e}")
            return None
    
    def calculate3DFaceBounds2D(self, car_3d, intrinsics):
        """Calculate 2D projection of the closest face of a 3D car bounding box"""
        try:
            # Get 3D vertices  
            vertices = self.getCuboidVertices(car_3d['bounding_box_3D'], car_3d.get('rotation'))
            
            # Find closest face
            closest_face_indices = self.findClosestFace(vertices)
            closest_face_vertices = vertices[closest_face_indices]
            
            # Project closest face to 2D
            projected_face = self.project3DTo2D(closest_face_vertices, intrinsics)
            if projected_face is None:
                return None
                
            # Get bounding rectangle of the face
            x_coords = projected_face[:, 0]
            y_coords = projected_face[:, 1]
            
            return {
                'x': int(np.min(x_coords)),
                'y': int(np.min(y_coords)), 
                'width': int(np.max(x_coords) - np.min(x_coords)),
                'height': int(np.max(y_coords) - np.min(y_coords))
            }
        except Exception as e:
            print(f"Error calculating 3D face bounds: {e}")
            return None
        
    def calculate3DOverlapScore(self, car_3d, plate_bbox, intrinsics, use_face_projection=True):
        """Calculate overlap score between 3D car bounding box and 2D license plate using Percebro methodology"""
        try:
            # Get 2D projection of car's 3D bounding box
            if use_face_projection:
                # Use closest face projection (like Percebro's approach)
                car_2d_bounds = self.calculate3DFaceBounds2D(car_3d, intrinsics)
            else:
                # Use full 3D box projection
                car_2d_bounds = self.calculate3DBounds2D(car_3d, intrinsics)
                
            if car_2d_bounds is None:
                return 0.0
                
            # Extract license plate coordinates
            plate_x1, plate_y1 = plate_bbox['x'], plate_bbox['y']
            plate_x2, plate_y2 = plate_x1 + plate_bbox['width'], plate_y1 + plate_bbox['height']
            plate_center_x = (plate_x1 + plate_x2) / 2
            plate_center_y = (plate_y1 + plate_y2) / 2
            
            # Extract car projected coordinates
            car_x1, car_y1 = car_2d_bounds['x'], car_2d_bounds['y']
            car_x2, car_y2 = car_x1 + car_2d_bounds['width'], car_y1 + car_2d_bounds['height']
            
            # Expand car bounds by buffer to account for license plates on bumpers
            car_width = car_x2 - car_x1
            car_height = car_y2 - car_y1
            margin_x = car_width * VEHICLE_BOUNDS_BUFFER
            margin_y = car_height * VEHICLE_BOUNDS_BUFFER
            
            expanded_car_x1 = car_x1 - margin_x
            expanded_car_y1 = car_y1 - margin_y
            expanded_car_x2 = car_x2 + margin_x
            expanded_car_y2 = car_y2 + margin_y
            
            # Check if plate center is within expanded car bounds
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
            print(f"Error calculating 3D overlap score: {e}")
            return 0.0

    def associateLicensePlates(self, objects, intrinsics=None):
        """Create associations between cars and license plates using 3D bounding boxes (Percebro methodology)"""
        cars = objects.get('car', [])
        license_plates = objects.get('license_plate', [])
        
        if not cars or not license_plates:
            return []
        
        used_plates = set()
        associations_created = 0
        inference_keys = ['license_plate']
        
        for car_idx, car in enumerate(cars):
            associated_plates = []
            
            # Check if car has 3D bounding box data
            has_3d_data = ('bounding_box_3D' in car or 
                          ('translation' in car and 'rotation' in car and 'size' in car))
            
            if has_3d_data and intrinsics is not None:
                # Use 3D-based association (Percebro methodology)
                # Ensure car has proper 3D data structure
                if 'bounding_box_3D' not in car and 'translation' in car:
                    car['bounding_box_3D'] = {
                        'x': car['translation'][0],
                        'y': car['translation'][1], 
                        'z': car['translation'][2],
                        'width': car['size'][0],
                        'height': car['size'][1],
                        'depth': car['size'][2]
                    }
                
                # Find best matching license plates using 3D projection
                plate_scores = []
                for plate_idx, plate in enumerate(license_plates):
                    if plate_idx in used_plates:
                        continue
                        
                    plate_bbox = plate['bounding_box_px']
                    # Use face-based projection for more accurate association
                    score = self.calculate3DOverlapScore(car, plate_bbox, intrinsics, use_face_projection=True)
                    if score > 0.1:  # Minimum threshold
                        plate_scores.append((score, plate_idx, plate))
                        
            else:
                # Fallback to 2D-based association for cars without 3D data
                if 'bounding_box_px' not in car:
                    continue
                    
                car_bbox = car['bounding_box_px']
                plate_scores = []
                for plate_idx, plate in enumerate(license_plates):
                    if plate_idx in used_plates:
                        continue
                        
                    plate_bbox = plate['bounding_box_px']
                    score = self.calculate2DOverlapScore(car_bbox, plate_bbox)
                    if score > 0.1:  # Minimum threshold
                        plate_scores.append((score, plate_idx, plate))
            
            # Sort by score and take the best matches
            plate_scores.sort(reverse=True, key=lambda x: x[0])
            
            for score, plate_idx, plate in plate_scores[:2]:  # Max 2 plates per car
                # Extract OCR text
                plate_info = {
                    'bounding_box_px': plate['bounding_box_px'],
                    'confidence': plate['confidence'],
                    'text': plate.get('text', '')
                }
                associated_plates.append(plate_info)
                used_plates.add(plate_idx)
                associations_created += 1
                
            # Add license plates to car object
            if associated_plates:
                car['license_plates'] = associated_plates
        
        # Remove associated license plates from the main objects list
        if used_plates:
            remaining_plates = [plate for idx, plate in enumerate(license_plates) if idx not in used_plates]
            if remaining_plates:
                objects['license_plate'] = remaining_plates
            else:
                # Remove license_plate category entirely if all plates were associated
                if 'license_plate' in objects:
                    del objects['license_plate']
        
        self.associations_created = associations_created
        print(f"3D Association: Created {associations_created} car-license plate associations")
        return inference_keys
    
    def calculate2DOverlapScore(self, car_bbox, plate_bbox):
        """Fallback 2D overlap calculation for cars without 3D data"""
        try:
            # Extract coordinates
            car_x1, car_y1 = car_bbox['x'], car_bbox['y']
            car_x2, car_y2 = car_x1 + car_bbox['width'], car_y1 + car_bbox['height']
            
            plate_x1, plate_y1 = plate_bbox['x'], plate_bbox['y']
            plate_x2, plate_y2 = plate_x1 + plate_bbox['width'], plate_y1 + plate_bbox['height']
            
            # Check if plate center is within expanded car bounds
            plate_center_x = (plate_x1 + plate_x2) / 2
            plate_center_y = (plate_y1 + plate_y2) / 2
            
            # Expand car bounds to account for license plates on bumpers
            car_width = car_x2 - car_x1
            car_height = car_y2 - car_y1
            margin_x = car_width * VEHICLE_BOUNDS_BUFFER
            margin_y = car_height * VEHICLE_BOUNDS_BUFFER
            
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
            print(f"Error calculating 2D overlap score: {e}")
            return 0.0

    def annotateCarLicensePlates(self, img, objects, obj_colors, intrinsics=None):
        """Annotate cars and their associated license plates (supports both 2D and 3D)"""
        cars = objects.get('car', [])
        
        for obj in cars:
            # Check if this is a 3D object
            has_3d_data = ('bounding_box_3D' in obj or 
                          ('translation' in obj and 'rotation' in obj and 'size' in obj))
            
            if has_3d_data and intrinsics is not None:
                # Draw 3D car bounding box
                self.annotate3DCarWithPlates(img, obj, obj_colors, intrinsics)
            else:
                # Draw 2D car bounding box (fallback)
                if 'bounding_box_px' in obj:
                    topleft_cv = (int(obj['bounding_box_px']['x']), int(obj['bounding_box_px']['y']))
                    bottomright_cv = (int(obj['bounding_box_px']['x'] + obj['bounding_box_px']['width']),
                                    int(obj['bounding_box_px']['y'] + obj['bounding_box_px']['height']))
                    cv2.rectangle(img, topleft_cv, bottomright_cv, obj_colors[1], 2)  # Car color
                
                # Annotate associated license plates
                if 'license_plates' in obj:
                    for plate in obj['license_plates']:
                        # Draw license plate bounding box
                        plate_topleft = (int(plate['bounding_box_px']['x']), int(plate['bounding_box_px']['y']))
                        plate_bottomright = (int(plate['bounding_box_px']['x'] + plate['bounding_box_px']['width']),
                                           int(plate['bounding_box_px']['y'] + plate['bounding_box_px']['height']))
                        cv2.rectangle(img, plate_topleft, plate_bottomright, obj_colors[3], 2)  # License plate color
                        
                        # Draw OCR text
                        if 'text' in plate and plate['text']:
                            self.annotatePlate(img, plate['bounding_box_px'], plate['text'])
    
    def annotate3DCarWithPlates(self, img, car_obj, obj_colors, intrinsics):
        """Annotate 3D car with its associated license plates"""
        try:
            # Ensure car has proper 3D data structure
            if 'bounding_box_3D' not in car_obj and 'translation' in car_obj:
                car_obj['bounding_box_3D'] = {
                    'x': car_obj['translation'][0],
                    'y': car_obj['translation'][1], 
                    'z': car_obj['translation'][2],
                    'width': car_obj['size'][0],
                    'height': car_obj['size'][1],
                    'depth': car_obj['size'][2]
                }
            
            # Draw 3D car bounding box
            self.annotate3DObject(img, car_obj, intrinsics, color=obj_colors[1])
            
            # Annotate associated license plates
            if 'license_plates' in car_obj:
                for plate in car_obj['license_plates']:
                    # Draw license plate bounding box
                    plate_topleft = (int(plate['bounding_box_px']['x']), int(plate['bounding_box_px']['y']))
                    plate_bottomright = (int(plate['bounding_box_px']['x'] + plate['bounding_box_px']['width']),
                                       int(plate['bounding_box_px']['y'] + plate['bounding_box_px']['height']))
                    cv2.rectangle(img, plate_topleft, plate_bottomright, obj_colors[3], 2)  # License plate color
                    
                    # Draw OCR text
                    if 'text' in plate and plate['text']:
                        self.annotatePlate(img, plate['bounding_box_px'], plate['text'])
                        
        except Exception as e:
            print(f"Error annotating 3D car: {e}")
    
    def annotate3DObject(self, img, obj, intrinsics, color=(66, 186, 150), thickness=2):
        """Annotate 3D object on the image"""
        try:
            vertex_idxs = [0, 1, 2, 3, 7, 6, 5, 4, 7, 3, 0, 4, 5, 1, 2, 6]
            rotation = obj.get('rotation')
            
            # Create cuboid vertices based on 3D bounding box
            vertices = self.getCuboidVertices(obj['bounding_box_3D'], rotation)
            
            # Project to 2D
            transformed_vertices = self.project3DTo2D(vertices, intrinsics)
            if transformed_vertices is None:
                return
            
            # Draw the 3D bounding box
            for idx in range(len(vertex_idxs)-1):
                if (vertex_idxs[idx] < len(transformed_vertices) and 
                    vertex_idxs[idx+1] < len(transformed_vertices)):
                    cv2.line(img,
                            tuple(transformed_vertices[vertex_idxs[idx]]),
                            tuple(transformed_vertices[vertex_idxs[idx+1]]),
                            color=(255,0,0) if idx == 0 else color,
                            thickness=thickness)
        except Exception as e:
            print(f"Error annotating 3D object: {e}")

    def annotatePlate(self, frame, bounds, text):
        """Annotate license plate text near the bounding box"""
        # Calculate text scale based on plate width for readability
        scale = 1
        lsize = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1*scale, 5*scale)[0]

        if lsize[0] > 0:
            scale = scale * 2 * bounds['width'] / lsize[0]
        lsize = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1*scale, int(5*scale))[0]

        start_x = int(bounds['x'] - lsize[0])
        bottom_y = int(bounds['y'] + 10 + lsize[1])
        end_x = int(bounds['x'])
        top_y = int(bounds['y'] + 10)
        
        # Check if annotation is within image bounds
        if self.pointsInsideImage(frame, [[start_x, top_y], [end_x, bottom_y]]):
            # Draw text with black outline and white fill for better visibility
            cv2.putText(frame, text, (start_x, bottom_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 1 * scale, (0,0,0), int(5 * scale))
            cv2.putText(frame, text, (start_x, bottom_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 1 * scale, (255,255,255), int(2 * scale))

    def pointsInsideImage(self, frame, img_pts):
        """Check if points are within image boundaries"""
        frame_height, frame_width = frame.shape[:2]
        for point in img_pts:
            pt_x = int(point[0])
            pt_y = int(point[1])
            if pt_x < 0 or pt_x > frame_width or pt_y < 0 or pt_y > frame_height:
                return False
        return True
