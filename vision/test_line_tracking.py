"""Run with real OpenCV: python -B -m unittest test_line_tracking -v."""
import copy
import json
import math
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from line_tracking import LineTrackingService, pose_in_line, wrap
from measurement_geometry import cv2, np

ROOT = Path(__file__).resolve().parent


class GeometryTests(unittest.TestCase):
    def setup_value(self):
        return {'line_angle_rad': 0., 'mount_yaw_rad': 0., 'origin_plane_mm': [0., 0.],
                'offset_body_mm': [82., 1.], 'offset_confirmed': True}

    def test_offset_rotates_with_robot(self):
        s = self.setup_value()
        a = pose_in_line([0, 0], 0, s)
        b = pose_in_line([0, 0], math.pi/2, s)
        self.assertAlmostEqual(a['x_mm'], 82)
        self.assertAlmostEqual(b['x_mm'], -1)
        self.assertAlmostEqual(b['y_mm'], 82)
        self.assertAlmostEqual(b['heading_deg'], 90)

    def test_left_positive_with_rotated_line(self):
        s = self.setup_value()
        s.update(line_angle_rad=math.pi/2, offset_confirmed=False)
        p = pose_in_line([-10, 100], math.pi/2, s)
        self.assertAlmostEqual(p['x_mm'], 100)
        self.assertAlmostEqual(p['y_mm'], 10)
        self.assertAlmostEqual(p['heading_deg'], 0)
        self.assertEqual(p['tracked_point'], 'marker_center')

    def test_branch_cut(self):
        self.assertAlmostEqual(math.degrees(wrap(math.radians(359))), -1)


class ServiceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        saved = json.loads((ROOT/'data'/'line_resume_candidate.json').read_text(encoding='utf-8-sig'))
        self.plane = saved['plane']
        from types import SimpleNamespace
        candidate = json.loads((ROOT/'data'/saved['session_id']/'session.json').read_text(encoding='utf-8-sig'))['calibration']
        self.app = SimpleNamespace(lock=threading.RLock(), action_lock=threading.Lock(), quit=threading.Event(),
                                   calibration=candidate, session_id=saved['session_id'], busy=False)
        self.epoch = self.plane['camera_epoch']
        self.counter = 0
        self.missing = False
        def raw():
            self.counter += 1
            return (np.zeros((1080,1920,3), dtype=np.uint8), self.counter, time.monotonic(), time.time(), self.epoch)
        self.app.plane_measurement = SimpleNamespace(_raw=raw, _current_plane=lambda *args: copy.deepcopy(self.plane))
        self.service = LineTrackingService(self.app, Path(self.temp.name))
        (Path(self.temp.name)/'data').mkdir()
        self.addCleanup(self.service.cancel)

    def mapped(self, *args, **kwargs):
        return {'detected': not self.missing, 'position_plane_mm': [100.,20.], 'yaw_rad': math.pi-.005,
                'reason_code': 'target_not_found' if self.missing else 'plane_position_estimated'}

    def configure(self):
        now = time.monotonic()
        self.service.recent.clear()
        for i in range(12):
            self.service.recent.append({**self.mapped(), 'mono': now-.03*i})
        # Generate true floor endpoints in the actual calibrated geometry.
        plane = self.plane
        R = np.array(plane['R_plane_to_camera'])
        t = np.array(plane['t_plane_to_camera_mm'])
        rv, _ = cv2.Rodrigues(R)
        obj = np.array([[0.,0.,-69.],[1000.,0.,-69.]])
        cal = plane['calibration']
        pts, _ = cv2.projectPoints(obj, rv, t, np.array(cal['camera_matrix']), np.array(cal['distortion_coefficients']))
        # Actual frame may not cover the +1000 endpoint: use two visible endpoints.
        obj[:,0] = [-200.,800.]
        pts, _ = cv2.projectPoints(obj, rv, t, np.array(cal['camera_matrix']), np.array(cal['distortion_coefficients']))
        return self.service.configure({'points_px': pts.reshape(2,2).tolist(), 'heading_aligned': True})

    def test_frozen_frame_configuration_and_missing_data_log(self):
        with patch('line_tracking.map_marker', side_effect=self.mapped):
            self.service.start({'marker_height_mm': 99})
            self.configure()
            time.sleep(.3)
            self.service.record({'note':'automated fake-camera test'})
            time.sleep(.3)
            self.missing = True
            time.sleep(.3)
            self.service.action('event', {'name':'manual_intervention'})
            self.service.action('stop', {})
            run = self.service.state()['last_run']
            self.assertGreater(run['samples'], run['detected'])
            rows = [json.loads(l) for l in (Path(run['directory'])/'records.jsonl').read_text().splitlines()]
            missing = [r for r in rows if not r['detected']]
            self.assertTrue(missing)
            self.assertTrue(all('x_mm' not in r for r in missing))
            self.assertTrue(all(r['elapsed_s'] >= 0 for r in rows))
            self.assertEqual(len({r['frame_id'] for r in rows}), len(rows))
            self.assertTrue((Path(run['directory'])/'events.json').exists())
            self.assertFalse(self.service.state()['setup']['offset_confirmed'])
            self.service.cancel()
            self.assertFalse(self.app.action_lock.locked())

    def test_epoch_change_stops_worker_and_releases_lock(self):
        with patch('line_tracking.map_marker', side_effect=self.mapped):
            self.service.start({'marker_height_mm':99})
            self.epoch += 1
            self.service.worker.join(2)
            self.assertFalse(self.service.worker.is_alive())
            self.assertFalse(self.app.action_lock.locked())
            self.assertIn('Camera', self.service.last_error)

    def test_restore_requires_explicit_fixed_camera_confirmation(self):
        with self.assertRaises(ValueError):
            self.service.restore({})
        self.assertFalse(self.app.action_lock.locked())

    def test_bad_height_does_not_hold_lock(self):
        with self.assertRaises(ValueError):
            self.service.start({'marker_height_mm':float('nan')})
        self.assertFalse(self.app.action_lock.locked())


if __name__ == '__main__':
    unittest.main()
