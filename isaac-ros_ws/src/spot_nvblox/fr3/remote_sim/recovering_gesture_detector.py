"""Original trained GestureRecognizer, single-hand and recoverable after a failed graph."""
import time


class RecoveringGestureDetector:
    def __init__(self, model_path=None, factory=None, image_factory=None, clock=time.monotonic):
        if factory is None:
            import mediapipe as mp
            def factory():
                options=mp.tasks.vision.GestureRecognizerOptions(
                    base_options=mp.tasks.BaseOptions(model_asset_path=str(model_path)),
                    running_mode=mp.tasks.vision.RunningMode.VIDEO,num_hands=1,
                    min_hand_detection_confidence=.5,min_hand_presence_confidence=.5,min_tracking_confidence=.3)
                return mp.tasks.vision.GestureRecognizer.create_from_options(options)
            image_factory=lambda rgb:mp.Image(image_format=mp.ImageFormat.SRGB,data=rgb)
        self.factory,self.image_factory,self.clock=factory,image_factory or (lambda rgb:rgb),clock
        self.engine=self.factory();self.retry_at=0.;self.restarts=0;self.last_error=None

    def recognize_for_video(self, rgb, timestamp_ms):
        if self.engine is None:
            if self.clock()<self.retry_at:raise RuntimeError('Reconhecedor de gestos reiniciando; controle em pausa')
            try:self.engine=self.factory();self.restarts+=1
            except (RuntimeError,ValueError):
                self.retry_at=self.clock()+1.
                raise RuntimeError('Reconhecedor indisponível; nova tentativa em 1 s')
        try:return self.engine.recognize_for_video(self.image_factory(rgb),timestamp_ms)
        except RuntimeError as error:
            self.last_error=str(error)
            self.close();self.retry_at=self.clock()+1.
            # Never return an older pose/gesture after a graph failure.
            raise RuntimeError('Falha do reconhecedor de gestos; reiniciando automaticamente') from error

    def close(self):
        if self.engine is not None:
            try:self.engine.close()
            except RuntimeError:pass
            self.engine=None
