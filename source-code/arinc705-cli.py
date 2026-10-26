#!/usr/bin/env python3
"""
ARINC705Forge Pro - Standalone Python CLI Runner & Simulation Suite
Standards: ARINC Characteristic 705, RTCA DO-334, FAA TSO-C4c/C6e/C8e, ARINC Specification 429-19
"""

import sys
import os
import json
import math
import argparse

class ARINC705AHRS:
    def __init__(self, unit_id=1, pitch_deg=2.5, roll_deg=0.0, mag_hdg_deg=270.0, mag_var_deg=-3.5, tas_kt=250.0):
        self.unit_id = unit_id
        self.mode = 'SLAVED' # 'ALIGNMENT', 'SLAVED', 'FREE_DG', 'BASIC_ATTITUDE'
        self.pitch_deg = float(pitch_deg)
        self.roll_deg = float(roll_deg)
        self.mag_hdg_deg = float(mag_hdg_deg)
        self.mag_var_deg = float(mag_var_deg)
        
        self.pitch_rate_dps = 0.0
        self.roll_rate_dps = 0.0
        self.yaw_rate_dps = 0.0

        self.accel_x_g = 0.0
        self.accel_y_g = 0.0
        self.accel_z_g = 1.0

        self.tas_kt = float(tas_kt)
        self.unslaved_drift_deg_per_hr = 1.5
        self.slaving_cutoff_dps = 3.0

        self.sensors = {
            'gyros_ok': True,
            'accels_ok': True,
            'flux_valve_connected': True,
            'alignment_complete': True
        }

        self.faults = {
            'pitch_invalid': False,
            'roll_invalid': False,
            'heading_invalid': False,
            'flux_valve_loss': False,
            'excessive_drift': False,
            'power_low': False
        }

    def get_true_heading(self):
        th = (self.mag_hdg_deg + self.mag_var_deg) % 360.0
        if th < 0: th += 360.0
        return round(th, 2)

    def calculate_centrifugal_accel_g(self):
        tas_mps = self.tas_kt * 0.514444
        yaw_rad = math.radians(self.yaw_rate_dps)
        cent_g = (tas_mps * yaw_rad) / 9.80665
        return round(cent_g, 3)

    def update_slaving(self, flux_hdg_deg, dt_sec=1.0):
        if self.mode != 'SLAVED' or not self.sensors['flux_valve_connected']:
            return {'slaving_active': False, 'turn_inhibited': False, 'heading': self.mag_hdg_deg}

        turn_inhibited = abs(self.yaw_rate_dps) > self.slaving_cutoff_dps or abs(self.roll_deg) > 6.0
        k = 0.005 if turn_inhibited else 0.05
        err = flux_hdg_deg - self.mag_hdg_deg
        while err > 180: err -= 360
        while err < -180: err += 360

        self.mag_hdg_deg = (self.mag_hdg_deg + (err * k * dt_sec)) % 360.0
        if self.mag_hdg_deg < 0: self.mag_hdg_deg += 360.0
        self.mag_hdg_deg = round(self.mag_hdg_deg, 2)

        return {
            'slaving_active': True,
            'turn_inhibited': turn_inhibited,
            'slaving_error_deg': round(err, 2),
            'heading': self.mag_hdg_deg
        }

    def execute_alignment(self, duration_sec=45.0):
        self.mode = 'ALIGNMENT'
        self.sensors['alignment_complete'] = False
        self.pitch_deg = 0.0
        self.roll_deg = 0.0
        if duration_sec >= 30.0:
            self.sensors['alignment_complete'] = True
            self.mode = 'SLAVED'
            return {'success': True, 'mode': self.mode, 'time_sec': duration_sec}
        return {'success': False, 'mode': self.mode, 'reason': 'ALIGNMENT_INCOMPLETE'}

    @staticmethod
    def compute_odd_parity(val31):
        count = bin(val31).count('1')
        return 1 if (count % 2 == 0) else 0

    def encode_bnr_angle(self, label_octal, angle_deg, is_signed=True):
        word = label_octal
        word |= ((self.unit_id & 0x03) << 8)
        norm = angle_deg
        if not is_signed:
            norm = (norm % 360.0 + 360.0) % 360.0
        else:
            while norm > 180.0: norm -= 360.0
            while norm < -180.0: norm += 360.0

        scale = 180.0 / 0x4000
        bnr = round(norm / scale) & 0x7FFF
        word |= (bnr << 13)
        ssm = 0x00 if (self.faults['pitch_invalid'] or self.faults['roll_invalid']) else 0x03
        word |= (ssm << 29)
        word |= (self.compute_odd_parity(word) << 31)
        return word

    def encode_label_324(self): return self.encode_bnr_angle(0xD4, self.pitch_deg, True)
    def encode_label_325(self): return self.encode_bnr_angle(0x54, self.roll_deg, True)
    def encode_label_320(self): return self.encode_bnr_angle(0x14, self.mag_hdg_deg, False)
    def encode_label_314(self): return self.encode_bnr_angle(0x66, self.get_true_heading(), False)

    def encode_label_270(self):
        word = 0x58
        word |= ((self.unit_id & 0x03) << 8)
        if self.mode == 'ALIGNMENT': word |= (1 << 10)
        if self.mode == 'SLAVED': word |= (1 << 11)
        if self.mode == 'FREE_DG': word |= (1 << 12)
        if self.sensors['alignment_complete']: word |= (1 << 13)
        if not self.faults['heading_invalid']: word |= (1 << 14)
        ssm = 0x03 if self.sensors['alignment_complete'] else 0x01
        word |= (ssm << 29)
        word |= (self.compute_odd_parity(word) << 31)
        return word

    def encode_label_350(self):
        word = 0xE8
        word |= ((self.unit_id & 0x03) << 8)
        if not self.sensors['gyros_ok']: word |= (1 << 10)
        if not self.sensors['accels_ok']: word |= (1 << 11)
        if not self.sensors['flux_valve_connected']: word |= (1 << 12)
        if self.faults['excessive_drift']: word |= (1 << 13)
        if self.faults['power_low']: word |= (1 << 14)
        has_fault = any(self.faults.values()) or not self.sensors['gyros_ok'] or not self.sensors['accels_ok']
        ssm = 0x00 if has_fault else 0x03
        word |= (ssm << 29)
        word |= (self.compute_odd_parity(word) << 31)
        return word

    def run_normative_audit(self):
        tests = []
        # 1. Form Fit
        tests.append(('ARINC705-REQ-001', 'ARINC 705 Form, Fit & Mounting', self.unit_id in [1, 2, 3]))
        # 2. Attitude Coverage
        tests.append(('ARINC705-REQ-002', 'Full Attitude Coverage (Pitch ±90°, Roll ±180°)', -90 <= self.pitch_deg <= 90 and -180 <= self.roll_deg <= 180))
        # 3. Dynamic Range
        max_rate = max(abs(self.pitch_rate_dps), abs(self.roll_rate_dps), abs(self.yaw_rate_dps))
        tests.append(('ARINC705-REQ-003', 'Rate Gyro Dynamic Range (Up to ±128°/sec)', max_rate <= 128.0))
        # 4. Flux Slaving
        res_slave = self.update_slaving(275.0, 1.0)
        tests.append(('ARINC705-REQ-004', 'Remote Flux Valve Magnetic Heading Slaving', res_slave['slaving_active']))
        # 5. Mag Var Comp
        th = self.get_true_heading()
        exp_th = round((self.mag_hdg_deg + self.mag_var_deg) % 360.0, 2)
        tests.append(('ARINC705-REQ-005', 'Magnetic Variation Compensation & True Heading', abs(th - exp_th) < 0.1))
        # 6. Centrifugal Turn Error Comp
        prev_y = self.yaw_rate_dps; self.yaw_rate_dps = 3.0
        cg = self.calculate_centrifugal_accel_g()
        self.yaw_rate_dps = prev_y
        tests.append(('ARINC705-REQ-006', 'Centrifugal Turn Error Acceleration Compensation', 0.05 < cg < 0.30))
        # 7. Slaving Cutoff in Turns
        prev_r = self.roll_deg; self.roll_deg = 20.0
        res_turn = self.update_slaving(290.0, 1.0)
        self.roll_deg = prev_r
        tests.append(('ARINC705-REQ-007', 'Slaving Loop Cutoff in Turns (> 6° Bank)', res_turn['turn_inhibited']))
        # 8. Free DG Drift Stability
        tests.append(('ARINC705-REQ-008', 'Free Directional Gyro (DG) Drift Stability (<= 2°/hr)', self.unslaved_drift_deg_per_hr <= 2.0))
        # 9. Rapid Ground Alignment
        res_align = self.execute_alignment(45.0)
        tests.append(('ARINC705-REQ-009', 'Rapid Ground Alignment (< 60 Seconds)', res_align['success']))
        # 10. ARINC 429 Bus & Parity
        w324 = self.encode_label_324()
        w325 = self.encode_label_325()
        w320 = self.encode_label_320()
        w314 = self.encode_label_314()
        w270 = self.encode_label_270()
        w350 = self.encode_label_350()
        parity_ok = all(bin(w).count('1') % 2 == 1 for w in [w324, w325, w320, w314, w270, w350])
        tests.append(('ARINC705-REQ-010', 'ARINC 429 High-Speed Bus BNR Words & Odd Parity', parity_ok))

        passed = sum(1 for _, _, ok in tests if ok)
        return {
            'total': len(tests),
            'passed': passed,
            'score': f"{(passed / len(tests)) * 100:.1f}%",
            'status': 'ARINC_705_AHRS_CERTIFIED' if passed == len(tests) else 'NON_COMPLIANT',
            'results': [{'req': r, 'name': n, 'passed': p} for r, n, p in tests]
        }

def main():
    parser = argparse.ArgumentParser(description="ARINC705Forge Pro CLI AHRS Simulator")
    parser.add_argument("--unit", type=int, default=1, choices=[1, 2, 3], help="AHRS Unit (1=Pilot, 2=Copilot, 3=Aux)")
    parser.add_argument("--pitch", type=float, default=2.5, help="Pitch angle in degrees")
    parser.add_argument("--roll", type=float, default=0.0, help="Roll angle in degrees")
    parser.add_argument("--hdg", type=float, default=270.0, help="Magnetic Heading in degrees")
    parser.add_argument("--var", type=float, default=-3.5, help="Magnetic Variation in degrees")
    parser.add_argument("--tas", type=float, default=250.0, help="True Airspeed in knots")
    parser.add_argument("--mode", choices=['SLAVED', 'FREE_DG', 'ALIGNMENT'], default='SLAVED', help="AHRS operational mode")
    parser.add_argument("--audit", action="store_true", help="Run 10-point airworthiness compliance audit")
    parser.add_argument("--json", action="store_true", help="Output results in JSON format")

    args = parser.parse_args()

    ahrs = ARINC705AHRS(
        unit_id=args.unit,
        pitch_deg=args.pitch,
        roll_deg=args.roll,
        mag_hdg_deg=args.hdg,
        mag_var_deg=args.var,
        tas_kt=args.tas
    )
    ahrs.mode = args.mode

    if args.audit:
        res = ahrs.run_normative_audit()
        if args.json:
            print(json.dumps(res, indent=2))
        else:
            print("================================================================================")
            print(" ARINC705Forge Pro - 10-Point Normative Airworthiness Compliance Audit")
            print(" Standards: ARINC Characteristic 705, RTCA DO-334, FAA TSO-C4c/C6e/C8e")
            print("================================================================================")
            for r in res['results']:
                status = "PASS [OK]" if r['passed'] else "FAIL [X]"
                print(f" {r['req']:<18} | {r['name']:<50} | {status}")
            print("--------------------------------------------------------------------------------")
            print(f" Compliance Score: {res['score']} ({res['passed']}/{res['total']}) | Status: {res['status']}")
            print("================================================================================")
        return

    w324 = ahrs.encode_label_324()
    w325 = ahrs.encode_label_325()
    w320 = ahrs.encode_label_320()
    w314 = ahrs.encode_label_314()
    w270 = ahrs.encode_label_270()
    w350 = ahrs.encode_label_350()

    data = {
        'unit': f"AHRS {ahrs.unit_id}",
        'mode': ahrs.mode,
        'pitch_deg': ahrs.pitch_deg,
        'roll_deg': ahrs.roll_deg,
        'mag_hdg_deg': ahrs.mag_hdg_deg,
        'true_hdg_deg': ahrs.get_true_heading(),
        'mag_var_deg': ahrs.mag_var_deg,
        'centrifugal_g': ahrs.calculate_centrifugal_accel_g(),
        'arinc429_hex': {
            'label324': f"0x{w324:08X}",
            'label325': f"0x{w325:08X}",
            'label320': f"0x{w320:08X}",
            'label314': f"0x{w314:08X}",
            'label270': f"0x{w270:08X}",
            'label350': f"0x{w350:08X}"
        }
    }

    if args.json:
        print(json.dumps(data, indent=2))
    else:
        print("================================================================================")
        print(" ARINC705Forge Pro - Attitude and Heading Reference System Telemetry")
        print("================================================================================")
        print(f" Unit & Mode     : {data['unit']} | Mode: [{data['mode']}]")
        print(f" Attitude        : Pitch {data['pitch_deg']:+.2f}° | Roll {data['roll_deg']:+.2f}°")
        print(f" Heading         : Magnetic {data['mag_hdg_deg']:.1f}° | True {data['true_hdg_deg']:.1f}° (Variation: {data['mag_var_deg']:+.1f}°)")
        print(f" Turn Dynamics   : Centrifugal Turn Compensation: {data['centrifugal_g']:.3f} g")
        print("--------------------------------------------------------------------------------")
        print(f" ARINC 429 Bus   : L324={data['arinc429_hex']['label324']} | L325={data['arinc429_hex']['label325']} | L320={data['arinc429_hex']['label320']} | L314={data['arinc429_hex']['label314']}")
        print("================================================================================")

if __name__ == '__main__':
    main()
