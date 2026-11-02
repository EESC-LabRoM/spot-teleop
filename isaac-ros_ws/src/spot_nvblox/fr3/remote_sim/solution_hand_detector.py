"""MediaPipe Hands adapter: landmarks plus geometric intent, without the Tasks gesture graph."""
from types import SimpleNamespace
import numpy as np
from hand_tracking_math import finger_distance_signals


class SolutionHandDetector:
    def __init__(self, engine=None, unmirrored=True):
        self.unmirrored = unmirrored
        if engine is None:
            import mediapipe as mp
            engine = mp.solutions.hands.Hands(static_image_mode=False, max_num_hands=2,
                model_complexity=1, min_detection_confidence=.5, min_tracking_confidence=.4)
        self.engine = engine

    def recognize_for_video(self, rgb, timestamp_ms):
        result = self.engine.process(rgb)
        output = SimpleNamespace(hand_landmarks=[], hand_world_landmarks=[], handedness=[], gestures=[])
        for hand, world, label in zip(result.multi_hand_landmarks or [],
                result.multi_hand_world_landmarks or [], result.multi_handedness or []):
            category = label.classification[0]
            name = category.label
            # Hands defines handedness for mirrored selfie images; ZED RGB is unmirrored.
            if self.unmirrored: name = {'Left': 'Right', 'Right': 'Left'}.get(name, name)
            output.hand_landmarks.append(hand.landmark)
            output.hand_world_landmarks.append(world.landmark)
            output.handedness.append([SimpleNamespace(category_name=name, score=float(category.score))])
            signals = finger_distance_signals(np.array([[p.x, p.y, p.z] for p in world.landmark]))
            gesture = signals['gesture'] if signals else None
            output.gestures.append([SimpleNamespace(category_name=gesture, score=1.)] if gesture else [])
        return output

    def close(self):
        self.engine.close()


def remap_crop_landmarks(result,box,image_shape):
    """Only normalized image coordinates change; metric hand axes remain camera-aligned."""
    left,top,right,bottom=box;h,w=image_shape[:2]
    for hand in result.hand_landmarks:
        for point in hand:
            point.x=(left+point.x*(right-left))/w
            point.y=(top+point.y*(bottom-top))/h
    return result
