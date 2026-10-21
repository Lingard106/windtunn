"""Localized recovery of an unconfirmed OpenMV ROI in the new partition.

Thresholding, blob selection, confirmation and locked-loss behavior are inherited.
No scene, target state, simulator clock or extra camera acquisition is used.
"""
from __future__ import annotations

from pathlib import Path
import sys

# Match run.py/DartViewer's plain module identity: importing a second
# prototypes.vision_settings would make equivalent settings fail isinstance.
_PROTOTYPES = Path(__file__).resolve().parents[1] / "prototypes"
if str(_PROTOTYPES) not in sys.path:
    sys.path.insert(0, str(_PROTOTYPES))
from openmv_tracker import OpenMVTracker


class ReacquiringTracker(OpenMVTracker):
    """Return to full-frame search after a miss before confirmation.

    The base tracker shrinks its ROI after the first detection, but only its
    confirmed-loss branch can restore full-frame search. A target that leaves
    this ROI before the three-frame confirmation can therefore be lost forever.
    This wrapper repairs only that missing unconfirmed-loss transition.
    """

    FULL_FRAME_ROI = (0, 0, 320, 240)

    def reset(self):
        super().reset()
        self._unconfirmed_recovery_count = 0

    def process(self, rgb):
        # Exactly one detector/state-machine update per supplied camera image.
        annotated, details = super().process(rgb)
        recovered = (not details["detected"] and not self.confirmed
                     and tuple(self.roi) != self.FULL_FRAME_ROI)
        if not details["detected"] and not self.confirmed:
            self.roi = self.FULL_FRAME_ROI
        if recovered:
            self._unconfirmed_recovery_count += 1
            details["roi"] = list(self.roi)
            # Update only the display annotation to match the next search ROI;
            # annotate does not threshold, process a frame, or alter tracker state.
            annotated = self.annotate(rgb, details)
        details["tracker_variant"] = "unconfirmed_ROI_reacquisition_v1"
        details["unconfirmed_roi_recovered"] = recovered
        details["unconfirmed_roi_recovery_count"] = self._unconfirmed_recovery_count
        return annotated, details

    def metadata(self):
        return dict(
            model="OpenMVTracker_with_unconfirmed_ROI_reacquisition",
            version=1,
            base_detector="prototypes/openmv_tracker.py::OpenMVTracker",
            policy="after detected=False and confirmed=False, next frame searches full 320x240",
            unchanged="LAB/exposure settings, blob selection, min pixels/aspect, ROI prediction, three-frame confirmation, five-frame confirmed loss, UART",
            confirm_frames=self.CONFIRM_FRAMES,
            confirmed_loss_frames=self.MAX_LOST,
            unconfirmed_roi_recovery_count=self._unconfirmed_recovery_count,
            input="one supplied 320x240 RGB camera image per process call",
            target_truth_used=False,
            additional_camera_samples=0,
            additional_detector_updates=0,
        )
