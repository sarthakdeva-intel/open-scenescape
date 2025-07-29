# SPDX-FileCopyrightText: (C) 2024 - 2025 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

import cv2


class CarLicensePlateProcessor:
    """Utility class for processing car and license plate associations and annotations"""
    
    def __init__(self):
        self.associations_created = 0
        
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
        
        # print(f"Associating {len(cars)} cars with {len(license_plates)} license plates")
        
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
                    'text': plate.get('text', '')
                }
                associated_plates.append(plate_info)
                used_plates.add(plate_idx)
                associations_created += 1
                
                # print(f"Associated car {car_idx} with license plate {plate_idx} (score: {score:.2f}, text: '{plate.get('text', '')}')")
            
            # Add license plates to car object
            if associated_plates:
                car['license_plates'] = associated_plates
        
        # Remove associated license plates from the main objects list
        if used_plates:
            remaining_plates = [plate for idx, plate in enumerate(license_plates) if idx not in used_plates]
            if remaining_plates:
                objects['license_plate'] = remaining_plates
                # print(f"Kept {len(remaining_plates)} unassociated license plates in main objects list")
            else:
                # Remove license_plate category entirely if all plates were associated
                if 'license_plate' in objects:
                    del objects['license_plate']
                # print("All license plates were associated with cars, removed license_plate category from main objects list")
        
        # print(f"Created {associations_created} license plate associations")
        self.associations_created = associations_created

    def annotateCarLicensePlates(self, img, objects, obj_colors):
        """Annotate cars and their associated license plates"""
        cars = objects.get('car', [])
        
        for obj in cars:
            # Draw car bounding box
            topleft_cv = (int(obj['bounding_box_px']['x']), int(obj['bounding_box_px']['y']))
            bottomright_cv = (int(obj['bounding_box_px']['x'] + obj['bounding_box_px']['width']),
                            int(obj['bounding_box_px']['y'] + obj['bounding_box_px']['height']))
            cv2.rectangle(img, topleft_cv, bottomright_cv, obj_colors[1], 2)  # Car color
            
            # Annotate license plates
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
