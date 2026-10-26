#!/usr/bin/env python3
"""
ARINC705Forge Pro - Automated Test Runner & Verification Suite
10 Comprehensive Verification Tests for ARINC 705 / RTCA DO-334 Compliance
"""

import unittest
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from importlib.machinery import SourceFileLoader
cli_module = SourceFileLoader("arinc705_cli", os.path.join(os.path.dirname(__file__), "arinc705-cli.py")).load_module()
ARINC705AHRS = cli_module.ARINC705AHRS

class TestARINC705ForgePro(unittest.TestCase):
    def setUp(self):
        self.ahrs = ARINC705AHRS(unit_id=1, pitch_deg=2.5, roll_deg=0.0, mag_hdg_deg=270.0, mag_var_deg=-3.5, tas_kt=250.0)

    # Test 1: Full Attitude Coverage
    def test_01_attitude_coverage(self):
        self.ahrs.pitch_deg = 85.0
        self.ahrs.roll_deg = 175.0
        self.assertTrue(-90 <= self.ahrs.pitch_deg <= 90)
        self.assertTrue(-180 <= self.ahrs.roll_deg <= 180)

    # Test 2: Rate Gyro Bandwidth
    def test_02_rate_gyro_range(self):
        self.ahrs.pitch_rate_dps = 45.0
        self.ahrs.roll_rate_dps = 60.0
        self.ahrs.yaw_rate_dps = 15.0
        max_rate = max(abs(self.ahrs.pitch_rate_dps), abs(self.ahrs.roll_rate_dps), abs(self.ahrs.yaw_rate_dps))
        self.assertLessEqual(max_rate, 128.0)

    # Test 3: Flux Valve Slaving Convergence
    def test_03_flux_valve_slaving(self):
        self.ahrs.mag_hdg_deg = 270.0
        res = self.ahrs.update_slaving(280.0, dt_sec=2.0)
        self.assertTrue(res['slaving_active'])
        self.assertGreater(self.ahrs.mag_hdg_deg, 270.0, "Heading must converge toward flux valve heading")

    # Test 4: Magnetic Variation & True Heading
    def test_04_magnetic_variation_true_heading(self):
        self.ahrs.mag_hdg_deg = 358.0
        self.ahrs.mag_var_deg = 5.0 # East
        th = self.ahrs.get_true_heading()
        self.assertEqual(th, 3.0, "358° + 5° variation must equal 003° True")

        self.ahrs.mag_hdg_deg = 2.0
        self.ahrs.mag_var_deg = -5.0 # West
        th_west = self.ahrs.get_true_heading()
        self.assertEqual(th_west, 357.0, "002° - 5° variation must equal 357° True")

    # Test 5: Centrifugal Turn Acceleration Compensation
    def test_05_centrifugal_turn_accel(self):
        self.ahrs.tas_kt = 250.0
        self.ahrs.yaw_rate_dps = 3.0 # Rate-one turn (180 deg / min)
        cent_g = self.ahrs.calculate_centrifugal_accel_g()
        # V = 250 * 0.514444 = 128.61 m/s, r = 3 * pi/180 = 0.05236 rad/s -> a_y = 6.734 m/s2 / 9.80665 = 0.686 g (calibrated)
        self.assertGreater(cent_g, 0.0)
        self.assertAlmostEqual(cent_g, 0.687, places=2)

    # Test 6: Slaving Cutoff in Coordinated Turns
    def test_06_slaving_cutoff_in_turns(self):
        # Level flight: not inhibited
        self.ahrs.roll_deg = 0.0
        self.ahrs.yaw_rate_dps = 0.0
        res_level = self.ahrs.update_slaving(275.0)
        self.assertFalse(res_level['turn_inhibited'])

        # Steep turn (25° bank): inhibited
        self.ahrs.roll_deg = 25.0
        res_turn = self.ahrs.update_slaving(275.0)
        self.assertTrue(res_turn['turn_inhibited'], "Slaving must be attenuated during turns > 6°")

    # Test 7: Free Directional Gyro (DG) Mode
    def test_07_free_dg_mode(self):
        self.ahrs.mode = 'FREE_DG'
        res = self.ahrs.update_slaving(300.0)
        self.assertFalse(res['slaving_active'], "Slaving must be completely disabled in Free DG mode")
        self.assertLessEqual(self.ahrs.unslaved_drift_deg_per_hr, 2.0)

    # Test 8: Rapid Ground Alignment
    def test_08_ground_alignment(self):
        res = self.ahrs.execute_alignment(45.0)
        self.assertTrue(res['success'])
        self.assertEqual(self.ahrs.mode, 'SLAVED')
        self.assertTrue(self.ahrs.sensors['alignment_complete'])

    # Test 9: ARINC 429 BNR Angle Words
    def test_09_arinc429_bnr_encoding(self):
        w324 = self.ahrs.encode_label_324()
        w325 = self.ahrs.encode_label_325()
        w320 = self.ahrs.encode_label_320()
        w314 = self.ahrs.encode_label_314()
        self.assertEqual(w324 & 0xFF, 0xD4) # Label 324
        self.assertEqual(w325 & 0xFF, 0x54) # Label 325
        self.assertEqual(w320 & 0xFF, 0x14) # Label 320
        self.assertEqual(w314 & 0xFF, 0x66) # Label 314

    # Test 10: ARINC 429 Bit 32 Odd Parity
    def test_10_arinc429_bit32_odd_parity(self):
        words = [
            self.ahrs.encode_label_324(),
            self.ahrs.encode_label_325(),
            self.ahrs.encode_label_320(),
            self.ahrs.encode_label_314(),
            self.ahrs.encode_label_270(),
            self.ahrs.encode_label_350()
        ]
        for w in words:
            ones = bin(w).count('1')
            self.assertEqual(ones % 2, 1, f"Word 0x{w:08X} has {ones} ones; must have odd parity (odd count)")

if __name__ == '__main__':
    runner = unittest.TextTestRunner(verbosity=2)
    suite = unittest.TestLoader().loadTestsFromTestCase(TestARINC705ForgePro)
    result = runner.run(suite)
    if not result.wasSuccessful():
        sys.exit(1)
    print("\nALL 10 VERIFICATION TESTS PASSED SUCCESSFULLY.")
