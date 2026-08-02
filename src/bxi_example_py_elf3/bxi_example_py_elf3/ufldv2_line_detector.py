import os

from bxi_example_py_elf3.traditional_line_detector import TraditionalLineDetector


class UFLDv2LineDetector:
    def __init__(self, model_path, image_width=640, image_height=360, dataset="auto"):
        self.model_path = model_path
        self.image_width = int(image_width)
        self.image_height = int(image_height)
        self.dataset = dataset
        self.fallback = TraditionalLineDetector(image_width, image_height)
        self.session = None

        if model_path and os.path.exists(model_path):
            try:
                import onnxruntime as ort

                self.session = ort.InferenceSession(
                    model_path,
                    providers=["CPUExecutionProvider"],
                )
            except Exception:
                self.session = None

    def detect(self, image):
        # Keep a stable fallback path. The UFLDv2 integration can be extended
        # once model preprocessing/output decoding is fixed for this track.
        return self.fallback.detect(image)
