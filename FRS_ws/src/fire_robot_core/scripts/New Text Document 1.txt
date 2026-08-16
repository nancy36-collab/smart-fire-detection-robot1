#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Professional Fire Detection System using Traditional Image Processing and OpenCV.
Optimized for IP Webcam with Dynamic Size/Rotation Mismatch Protection.
Upgraded with Exact Convex-Hull Bounding, Perfect Centroid Alignment, 
and Spatial Distance Gating for Terminal Logging.
Fully integrated with ROS 1 PointStamped Publisher.

Interactive Keybindings:
- 'q' or 'ESC' : Quit
- 'a'          : Toggle Auto-Adaptation Mode
- 'r'          : Rotate Frame (0 -> 90 -> 180 -> 270 degrees)
- 'h'          : Toggle Horizontal Flip (IP Webcam correction)
- 'v'          : Toggle Vertical Flip (IP Webcam correction)
"""

import rospy
from geometry_msgs.msg import PointStamped
import cv2
import numpy as np
import time
import argparse
import threading

class ThreadedVideoCapture:
    def __init__(self, source=0, width=640, height=480):
        if isinstance(source, str) and source.startswith("http"):
            if ".8080" in source:
                source = source.replace(".8080", ":8080")
            if not source.endswith("/video") and not source.endswith("/videofeed"):
                source = source.rstrip("/") + "/video"
            print(f"[SYSTEM INFO] Sanitized IP Webcam URL to: {source}")

        try:
            self.source = int(source)
        except ValueError:
            self.source = source

        self.cap = cv2.VideoCapture(self.source)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if isinstance(self.source, int):
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        self.grabbed, self.frame = self.cap.read()
        self.started = False
        self.read_lock = threading.Lock()
        
        self.rotation = 0  
        self.flip_h = False
        self.flip_v = False

    def start(self):
        if self.started:
            return self
        self.started = True
        self.thread = threading.Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()
        return self

    def update(self):
        while self.started:
            grabbed, frame = self.cap.read()
            if not grabbed:
                time.sleep(0.01)
                continue
            with self.read_lock:
                self.grabbed = grabbed
                self.frame = frame

    def read(self):
        with self.read_lock:
            frame = self.frame.copy() if self.frame is not None else None
            grabbed = self.grabbed
        
        if frame is not None:
            h, w = frame.shape[:2]
            if w > 640:
                scale = 640.0 / w
                frame = cv2.resize(frame, (640, int(h * scale)))

            if self.rotation == 90:
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            elif self.rotation == 180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            elif self.rotation == 270:
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            
            if self.flip_h and self.flip_v:
                frame = cv2.flip(frame, -1)
            elif self.flip_h:
                frame = cv2.flip(frame, 1)
            elif self.flip_v:
                frame = cv2.flip(frame, 0)
                
        return grabbed, frame

    def stop(self):
        self.started = False
        if hasattr(self, 'thread') and self.thread.is_alive():
            self.thread.join()
        self.cap.release()


class TrackedObject:
    def __init__(self, obj_id, centroid, bbox, area, perimeter, mean_intensity):
        self.id = obj_id
        self.centroid = centroid
        self.bbox = bbox
        self.history = []
        self.frames_since_seen = 0
        self.max_history = 15  
        self.is_confirmed_fire = False  
        self.update_history(centroid, bbox, area, perimeter, mean_intensity)

    def update_history(self, centroid, bbox, area, perimeter, mean_intensity):
        self.centroid = centroid
        self.bbox = bbox
        self.history.append({
            'area': area,
            'perimeter': perimeter,
            'mean_intensity': mean_intensity,
            'centroid': centroid
        })
        if len(self.history) > self.max_history:
            self.history.pop(0)
        self.frames_since_seen = 0


class CentroidTracker:
    def __init__(self, max_disappeared=5):
        self.next_id = 0
        self.objects = {}
        self.max_disappeared = max_disappeared

    def update(self, candidates):
        if len(candidates) == 0:
            for obj_id in list(self.objects.keys()):
                self.objects[obj_id].frames_since_seen += 1
                if self.objects[obj_id].frames_since_seen > self.max_disappeared:
                    del self.objects[obj_id]
            return self.objects

        if len(self.objects) == 0:
            for cand in candidates:
                self.objects[self.next_id] = TrackedObject(
                    self.next_id, cand['centroid'], cand['bbox'], 
                    cand['area'], cand['perimeter'], cand['mean_intensity']
                )
                self.next_id += 1
            return self.objects

        object_ids = list(self.objects.keys())
        object_centroids = np.array([self.objects[oid].centroid for oid in object_ids])
        candidate_centroids = np.array([cand['centroid'] for cand in candidates])

        distances = np.linalg.norm(object_centroids[:, np.newaxis] - candidate_centroids, axis=2)

        rows = distances.min(axis=1).argsort()
        cols = distances.argmin(axis=1)[rows]

        used_rows = set()
        used_cols = set()

        for r, c in zip(rows, cols):
            if r in used_rows or c in used_cols:
                continue
            if distances[r, c] > 80:
                continue
            
            obj_id = object_ids[r]
            cand = candidates[c]
            self.objects[obj_id].update_history(
                cand['centroid'], cand['bbox'], cand['area'], cand['perimeter'], cand['mean_intensity']
            )
            used_rows.add(r)
            used_cols.add(c)

        unused_rows = set(range(len(object_ids))) - used_rows
        for r in unused_rows:
            obj_id = object_ids[r]
            self.objects[obj_id].frames_since_seen += 1
            if self.objects[obj_id].frames_since_seen > self.max_disappeared:
                del self.objects[obj_id]

        unused_cols = set(range(len(candidates))) - used_cols
        for c in unused_cols:
            cand = candidates[c]
            self.objects[self.next_id] = TrackedObject(
                self.next_id, cand['centroid'], cand['bbox'], 
                cand['area'], cand['perimeter'], cand['mean_intensity']
            )
            self.next_id += 1

        return self.objects


class FireDetector:
    def __init__(self):
        self.tracker = CentroidTracker(max_disappeared=5)
        self.prev_gray1 = None
        self.prev_gray2 = None
        
        self.stabilized_bbox = None      
        self.fire_miss_frames = 0        
        
        # --- Memory for Terminal Printing ---
        self.last_printed_centroid = None
        self.print_distance_threshold = 40.0

    def analyze_illumination(self, frame_gray, frame_hsv):
        mean_brightness = np.mean(frame_gray)
        contrast = np.std(frame_gray)
        mean_v = np.mean(frame_hsv[:, :, 2])
        
        overexposed_ratio = np.sum(frame_gray >= 225) / frame_gray.size
        
        if mean_v < 55:
            scene_type = "Low Light"
            gamma = 0.75
            clahe_clip = 3.0
            min_s = 90
            min_v = 100
            morph_kernel = 3
            motion_thresh = 12
        elif mean_v > 210 or overexposed_ratio > 0.20:
            scene_type = "Overexposed"
            gamma = 1.9
            clahe_clip = 1.0
            min_s = 170
            min_v = 220
            morph_kernel = 7
            motion_thresh = 45
        elif mean_v > 160:
            scene_type = "Bright Daylight"
            gamma = 1.4
            clahe_clip = 1.2
            min_s = 140
            min_v = 180
            morph_kernel = 5
            motion_thresh = 35
        else:
            scene_type = "Normal Light"
            gamma = 1.0
            clahe_clip = 2.0
            min_s = 110
            min_v = 130
            morph_kernel = 5
            motion_thresh = 22
            
        return {
            'scene_type': scene_type,
            'mean_brightness': mean_brightness,
            'contrast': contrast,
            'mean_v': mean_v,
            'overexposed_ratio': overexposed_ratio,
            'gamma': gamma,
            'clahe_clip': clahe_clip,
            'min_s': min_s,
            'min_v': min_v,
            'morph_kernel': morph_kernel,
            'motion_thresh': motion_thresh
        }

    def apply_adaptive_preprocessing(self, frame, gamma, clahe_clip):
        if abs(gamma - 1.0) > 0.05:
            inv_gamma = 1.0 / gamma
            lut = np.array([((i / 255.0) ** inv_gamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
            enhanced = cv2.LUT(frame, lut)
        else:
            enhanced = frame.copy()

        gray = cv2.cvtColor(enhanced, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=(8, 8))
        gray_enhanced = clahe.apply(gray)
        gray_blurred = cv2.GaussianBlur(gray_enhanced, (5, 5), 0)

        return enhanced, gray_blurred

    def generate_color_mask(self, frame_bgr, min_s, min_v):
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        ycbcr = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2YCrCb)
        bgr_b, bgr_g, bgr_r = cv2.split(frame_bgr)
        y_ch, cb_ch, cr_ch = cv2.split(ycbcr)
        s_ch = hsv[:, :, 1]

        lower_hsv = np.array([0, min_s, min_v])
        upper_hsv = np.array([38, 255, 255])
        hsv_mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

        # Added saturation gating to prevent daylight white reflection false positives
        ycbcr_mask = (y_ch >= cb_ch) & (cr_ch >= cb_ch) & (y_ch > 115) & (cr_ch > 125) & (s_ch >= max(80, min_s - 20))
        ycbcr_mask_uint8 = (ycbcr_mask.astype(np.uint8)) * 255

        rgb_mask = (bgr_r > bgr_g) & (bgr_g > bgr_b) & (bgr_r > 150) & (s_ch >= max(80, min_s - 20))
        rgb_mask_uint8 = (rgb_mask.astype(np.uint8)) * 255

        combined_color = cv2.bitwise_and(hsv_mask, cv2.bitwise_or(ycbcr_mask_uint8, rgb_mask_uint8))
        return combined_color

    def calculate_shape_score(self, solidity, extent, aspect_ratio, circularity):
        score = 0.0
        
        if 0.45 <= solidity <= 0.90:
            score += 10.0
        elif solidity > 0.95:
            score -= 5.0

        if circularity < 0.72:
            score += 5.0
        elif circularity > 0.85:
            score -= 5.0

        if 0.15 <= extent <= 0.80:
            score += 5.0
            
        if 0.15 <= aspect_ratio <= 3.8:
            score += 5.0
        else:
            score -= 5.0
            
        return max(0.0, min(score, 25.0))

    def calculate_flicker_score(self, history):
        if len(history) < 5:
            return 12.5

        areas = [h['area'] for h in history]
        perimeters = [h['perimeter'] for h in history]
        intensities = [h['mean_intensity'] for h in history]

        mean_area = np.mean(areas)
        var_area = np.var(areas) / (mean_area ** 2 + 1e-6) if mean_area > 0 else 0

        mean_peri = np.mean(perimeters)
        var_peri = np.var(perimeters) / (mean_peri ** 2 + 1e-6) if mean_peri > 0 else 0

        var_intensity = np.var(intensities)

        if var_area < 0.0006 and var_intensity < 0.8:
            return 0.0  

        score = 0.0
        if 0.0015 <= var_area <= 0.30:
            score += 10.0
        elif var_area > 0.30:
            score += 5.0

        if 0.0015 <= var_peri <= 0.30:
            score += 10.0

        if var_intensity > 2.0:
            score += 5.0

        return max(0.0, min(score, 25.0))

    def process_frame(self, frame, auto_adapt, manual_gamma, manual_s, manual_v, manual_motion, conf_threshold):
        h, w = frame.shape[:2]

        if self.prev_gray1 is None or self.prev_gray1.shape != (h, w):
            self.prev_gray1 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            self.prev_gray2 = None 
            return frame, 0, (0, 0), {}, (0, 0, 0, 0)

        if self.prev_gray2 is None or self.prev_gray2.shape != (h, w):
            self.prev_gray2 = self.prev_gray1.copy()
            self.prev_gray1 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            return frame, 0, (0, 0), {}, (0, 0, 0, 0)

        temp_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        temp_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        metrics = self.analyze_illumination(temp_gray, temp_hsv)

        if auto_adapt:
            gamma = metrics['gamma']
            clahe_clip = metrics['clahe_clip']
            min_s = metrics['min_s']
            min_v = metrics['min_v']
            motion_thresh = metrics['motion_thresh']
            morph_kernel_size = metrics['morph_kernel']
        else:
            gamma = manual_gamma
            clahe_clip = 2.0
            min_s = manual_s
            min_v = manual_v
            motion_thresh = manual_motion
            morph_kernel_size = 5

        enhanced_bgr, prep_gray = self.apply_adaptive_preprocessing(frame, gamma, clahe_clip)

        diff1 = cv2.absdiff(prep_gray, self.prev_gray1)
        diff2 = cv2.absdiff(self.prev_gray1, self.prev_gray2)
        motion_raw = cv2.bitwise_and(diff1, diff2)
        _, motion_mask = cv2.threshold(motion_raw, motion_thresh, 255, cv2.THRESH_BINARY)
        
        m_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        motion_mask = cv2.dilate(motion_mask, m_kernel, iterations=1)

        self.prev_gray2 = self.prev_gray1.copy()
        self.prev_gray1 = prep_gray.copy()

        color_mask = self.generate_color_mask(enhanced_bgr, min_s, min_v)

        candidate_mask = cv2.bitwise_and(color_mask, motion_mask)
        morph_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (morph_kernel_size, morph_kernel_size))
        candidate_mask = cv2.morphologyEx(candidate_mask, cv2.MORPH_OPEN, morph_kernel)
        candidate_mask = cv2.morphologyEx(candidate_mask, cv2.MORPH_CLOSE, morph_kernel)

        contours, _ = cv2.findContours(candidate_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        frame_candidates = []
        hsv_v_chan = temp_hsv[:, :, 2]

        for c in contours:
            area = cv2.contourArea(c)
            if area < 15:
                continue

            perimeter = cv2.arcLength(c, True)
            if perimeter == 0:
                continue

            x, y, w_box, h_box = cv2.boundingRect(c)
            aspect_ratio = float(w_box) / h_box
            extent = area / (w_box * h_box)
            
            hull = cv2.convexHull(c)
            hull_area = cv2.contourArea(hull)
            solidity = area / hull_area if hull_area > 0 else 0
            circularity = (4 * np.pi * area) / (perimeter ** 2)

            mask_temp = np.zeros_like(prep_gray)
            cv2.drawContours(mask_temp, [c], -1, 255, -1)
            mean_intensity = cv2.mean(hsv_v_chan, mask=mask_temp)[0]

            cx = int(x + w_box / 2.0)
            cy = int(y + h_box / 2.0)

            frame_candidates.append({
                'contour': c,
                'centroid': (cx, cy),
                'bbox': (x, y, w_box, h_box),
                'area': area,
                'perimeter': perimeter,
                'solidity': solidity,
                'extent': extent,
                'aspect_ratio': aspect_ratio,
                'circularity': circularity,
                'mean_intensity': mean_intensity
            })

        tracked_objects = self.tracker.update(frame_candidates)
        output_frame = frame.copy()
        confirmed_fires = []
        
        for obj_id, obj in tracked_objects.items():
            match = None
            for cand in frame_candidates:
                if cand['centroid'] == obj.centroid:
                    match = cand
                    break

            if match is None:
                if obj.is_confirmed_fire and len(obj.history) > 0:
                    confirmed_fires.append({
                        'obj_id': obj_id,
                        'centroid': obj.centroid,
                        'bbox': obj.bbox,
                        'contour': None,  
                        'confidence': conf_threshold  
                    })
                continue

            color_score = 25.0
            mask_c = np.zeros_like(motion_mask)
            cv2.drawContours(mask_c, [match['contour']], -1, 255, -1)
            overlap = cv2.bitwise_and(mask_c, motion_mask)
            overlap_p = cv2.countNonZero(overlap)
            contour_p = cv2.countNonZero(mask_c) + 1e-6
            motion_ratio = overlap_p / contour_p
            motion_score = min(25.0, motion_ratio * 25.0)

            shape_score = self.calculate_shape_score(
                match['solidity'], match['extent'], match['aspect_ratio'], match['circularity']
            )

            flicker_score = self.calculate_flicker_score(obj.history)

            if 15 <= match['area'] < 120:
                if 0.3 <= match['aspect_ratio'] <= 1.0 and 0.72 <= match['solidity'] <= 0.92:
                    shape_score = max(shape_score, 20.0)

            total_confidence = color_score + motion_score + shape_score + flicker_score
            
            if len(obj.history) < 5:
                total_confidence = min(40.0, total_confidence)

            if total_confidence >= conf_threshold:
                obj.is_confirmed_fire = True

            if total_confidence < 30.0:
                obj.is_confirmed_fire = False

            if obj.is_confirmed_fire:
                confirmed_fires.append({
                    'obj_id': obj_id,
                    'centroid': obj.centroid,
                    'bbox': obj.bbox,
                    'contour': match['contour'],
                    'confidence': max(total_confidence, conf_threshold)
                })

        final_cx, final_cy = 0, 0
        zones_count = 0
        master_centroid = (0, 0)
        out_bbox = (0, 0, 0, 0)

        if len(confirmed_fires) > 0:
            self.fire_miss_frames = 0  
            all_points = []
            
            for fire in confirmed_fires:
                if fire['contour'] is not None:
                    all_points.extend(fire['contour'])
                    cv2.drawContours(output_frame, [fire['contour']], -1, (0, 150, 255), 1)

            if len(all_points) > 0:
                all_points = np.array(all_points)
                raw_x, raw_y, raw_w, raw_h = cv2.boundingRect(all_points)
                
                pad = 15
                sb_x = max(0, raw_x - pad)
                sb_y = max(0, raw_y - pad)
                sb_w = min(output_frame.shape[1] - sb_x, raw_w + 2 * pad)
                sb_h = min(output_frame.shape[0] - sb_y, raw_h + 2 * pad)

                self.stabilized_bbox = (sb_x, sb_y, sb_w, sb_h)

        elif self.stabilized_bbox is not None:
            if self.fire_miss_frames < 10:
                self.fire_miss_frames += 1  
            else:
                self.stabilized_bbox = None
                self.last_printed_centroid = None

        if self.stabilized_bbox is not None:
            sb_x, sb_y, sb_w, sb_h = map(int, self.stabilized_bbox)
            zones_count = 1  
            out_bbox = (sb_x, sb_y, sb_w, sb_h)

            cv2.rectangle(output_frame, (sb_x, sb_y), (sb_x + sb_w, sb_y + sb_h), (0, 0, 255), 2)

            final_cx = int(sb_x + sb_w / 2)
            final_cy = int(sb_y + sb_h / 2)
            master_centroid = (final_cx, final_cy)

            cross_size = 10
            cv2.line(output_frame, (final_cx - cross_size, final_cy), (final_cx + cross_size, final_cy), (255, 255, 255), 2)
            cv2.line(output_frame, (final_cx, final_cy - cross_size), (final_cx, final_cy + cross_size), (255, 255, 255), 2)

            cv2.putText(output_frame, f"Center({final_cx}, {final_cy})", (final_cx + 12, final_cy - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2, cv2.LINE_AA)

            avg_conf = np.mean([f['confidence'] for f in confirmed_fires]) / 100.0 if len(confirmed_fires) > 0 else (conf_threshold / 100.0)
            conf_text = f"CONF: {avg_conf:.2f}"
            (text_w, text_h), _ = cv2.getTextSize(conf_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            
            safe_text_y = max(sb_y, text_h + 12)
            cv2.rectangle(output_frame, (sb_x, safe_text_y - text_h - 8), (sb_x + text_w + 10, safe_text_y), (255, 255, 255), -1)
            cv2.rectangle(output_frame, (sb_x, safe_text_y - text_h - 8), (sb_x + text_w + 10, safe_text_y), (0, 0, 0), 1)
            cv2.putText(output_frame, conf_text, (sb_x + 5, safe_text_y - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

            should_print = False
            if self.last_printed_centroid is None:
                should_print = True
            else:
                dist = np.linalg.norm(np.array(master_centroid) - np.array(self.last_printed_centroid))
                if dist > self.print_distance_threshold:
                    should_print = True

            if should_print:
                print("\n" + "="*70)
                print(f"🚨 [ALERT] Locked on Target!")
                print(f"📍 Flame Center Coordinates -> X: {final_cx}, Y: {final_cy}")
                print("="*70 + "\n")
                self.last_printed_centroid = master_centroid

        active_params = {
            'scene_type': metrics['scene_type'],
            'mean_brightness': metrics['mean_brightness'],
            'contrast': metrics['contrast'],
            'overexposed_ratio': metrics['overexposed_ratio'],
            'gamma': gamma,
            'min_s': min_s,
            'min_v': min_v,
            'motion_thresh': motion_thresh,
            'conf_thresh': conf_threshold,
            'auto_adapt': auto_adapt
        }

        return output_frame, zones_count, master_centroid, active_params, out_bbox

def empty_callback(val):
    pass

def main():
    rospy.init_node('fire_detection_publisher', anonymous=True)
    pub = rospy.Publisher('/fire_coordinates', PointStamped, queue_size=10)

    video_source = rospy.get_param('~video_source', 'http://192.168.43.178:8080/video')
    focal_length = rospy.get_param('~focal_length_px', 600.0)
    real_fire_height = rospy.get_param('~real_fire_height_m', 0.20)

    print("[SYSTEM INFO] Initializing Threaded Video Stream...")
    cap_stream = ThreadedVideoCapture(source=video_source).start()
    
    time.sleep(1.0)
    detector = FireDetector()

    cv2.namedWindow("Fire Detection System", cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow("Control Panel", cv2.WINDOW_AUTOSIZE)
    cv2.resizeWindow("Control Panel", 400, 300)

    cv2.createTrackbar("Auto-Adapt", "Control Panel", 1, 1, empty_callback)
    cv2.createTrackbar("Conf Thresh", "Control Panel", 55, 100, empty_callback)
    cv2.createTrackbar("Gamma x100", "Control Panel", 100, 300, empty_callback)
    cv2.createTrackbar("Min S", "Control Panel", 120, 255, empty_callback)
    cv2.createTrackbar("Min V", "Control Panel", 130, 255, empty_callback)
    cv2.createTrackbar("Motion Thresh", "Control Panel", 25, 100, empty_callback)

    prev_time = time.time()
    fps = 0.0

    print("[SYSTEM INFO] System initialized successfully. Processing frames.")
    print("[HOTKEYS] Press 'q' to Exit | 'a' to Toggle Auto-Adapt | 'r' to Rotate | 'h'/'v' to Flip.")

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        grabbed, frame = cap_stream.read()
        if not grabbed or frame is None:
            if not isinstance(cap_stream.source, int):
                time.sleep(0.05)
                continue
            else:
                break

        auto_adapt = cv2.getTrackbarPos("Auto-Adapt", "Control Panel") == 1
        conf_thresh = cv2.getTrackbarPos("Conf Thresh", "Control Panel")
        man_gamma = cv2.getTrackbarPos("Gamma x100", "Control Panel") / 100.0
        man_s = cv2.getTrackbarPos("Min S", "Control Panel")
        man_v = cv2.getTrackbarPos("Min V", "Control Panel")
        man_motion = cv2.getTrackbarPos("Motion Thresh", "Control Panel")

        processed_frame, zones_count, master_centroid, params, bbox = detector.process_frame(
            frame=frame,
            auto_adapt=auto_adapt,
            manual_gamma=man_gamma,
            manual_s=man_s,
            manual_v=man_v,
            manual_motion=man_motion,
            conf_threshold=conf_thresh
        )

        current_time = time.time()
        time_diff = current_time - prev_time
        prev_time = current_time
        fps = 0.9 * fps + 0.1 * (1.0 / (time_diff + 1e-6))

        # --- ROS 1 Publishing Block ---
        if zones_count > 0 and master_centroid != (0, 0):
            final_cx, final_cy = master_centroid
            sb_h = bbox[3] if bbox[3] > 0 else 30
            h_img, w_img = processed_frame.shape[:2]
            cx_img = w_img / 2.0

            target_x_m = (focal_length * real_fire_height) / float(sb_h)
            target_y_m = -((final_cx - cx_img) * target_x_m) / focal_length

            msg = PointStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "base_link"
            msg.point.x = float(target_x_m)
            msg.point.y = float(target_y_m)
            msg.point.z = 0.0

            pub.publish(msg)

        # On-Screen Overlay Text Display
        cv2.rectangle(processed_frame, (5, 5), (320, 165), (20, 20, 20), -1)
        
        cv2.putText(processed_frame, f"FPS: {fps:.1f}", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)
        
        cv2.putText(processed_frame, f"Scene: {params.get('scene_type', 'N/A')}", (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        
        cv2.putText(processed_frame, f"Auto-Adaptation: {'ENABLED' if params.get('auto_adapt') else 'DISABLED'}", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 200, 0) if params.get('auto_adapt') else (0, 0, 255), 1, cv2.LINE_AA)

        cv2.putText(processed_frame, f"Gamma: {params.get('gamma', 1.0):.2f}", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1, cv2.LINE_AA)
        
        cv2.putText(processed_frame, f"HSV Bounds: H(0-38) S({params.get('min_s', 0)}-255) V({params.get('min_v', 0)}-255)", (10, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1, cv2.LINE_AA)

        cv2.putText(processed_frame, f"Motion Cutoff Thresh: {params.get('motion_thresh', 25)}", (10, 95),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1, cv2.LINE_AA)

        cv2.putText(processed_frame, f"Confidence Gate: {params.get('conf_thresh', 55)}%", (10, 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1, cv2.LINE_AA)

        cv2.putText(processed_frame, f"Brightness / Contrast: {params.get('mean_brightness', 0):.1f} / {params.get('contrast', 0):.1f}", (10, 125),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1, cv2.LINE_AA)

        rot_labels = {0: "0", 90: "90 CW", 180: "180", 270: "90 CCW"}
        flip_h_status = "ON" if cap_stream.flip_h else "OFF"
        flip_v_status = "ON" if cap_stream.flip_v else "OFF"
        cv2.putText(processed_frame, f"Rotation: {rot_labels[cap_stream.rotation]} | H-Flip: {flip_h_status} | V-Flip: {flip_v_status}", (10, 140),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)

        if zones_count > 0:
            status_color = (0, 0, 255)
            status_text = "🚨 WARNING: ACTIVE FIRE DETECTED!"
        else:
            status_color = (0, 255, 0)
            status_text = "STATUS: SCENE SECURE"

        cv2.putText(processed_frame, status_text, (10, 158),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, status_color, 1, cv2.LINE_AA)

        cv2.imshow("Fire Detection System", processed_frame)

        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q') or key == 27:
            print("[SYSTEM INFO] Termination requested by user.")
            break
        
        elif key == ord('a'):
            next_state = 0 if auto_adapt else 1
            cv2.setTrackbarPos("Auto-Adapt", "Control Panel", next_state)
            print(f"[SYSTEM INFO] Auto-adaptation toggled to: {'ON' if next_state == 1 else 'OFF'}")

        elif key == ord('r'):
            cap_stream.rotation = (cap_stream.rotation + 90) % 360
            print(f"[STREAM UPDATE] Frame rotation set to: {cap_stream.rotation} degrees.")

        elif key == ord('h'):
            cap_stream.flip_h = not cap_stream.flip_h
            print(f"[STREAM UPDATE] Horizontal Flip: {'ENABLED' if cap_stream.flip_h else 'DISABLED'}.")

        elif key == ord('v'):
            cap_stream.flip_v = not cap_stream.flip_v
            print(f"[STREAM UPDATE] Vertical Flip: {'ENABLED' if cap_stream.flip_v else 'DISABLED'}.")

        rate.sleep()

    print("[SYSTEM INFO] Stopping capture threads and closing windows.")
    cap_stream.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
