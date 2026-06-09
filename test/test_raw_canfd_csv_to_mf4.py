#!/usr/bin/env python3

import os
import subprocess
import tempfile
import unittest


class RawCanFdCsvToMf4Test(unittest.TestCase):
    def test_converter_writes_mf4_or_reports_missing_asammdf(self):
        package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        script = os.path.join(package_dir, "scripts", "raw_canfd_csv_to_mf4.py")
        with tempfile.TemporaryDirectory() as tmpdir:
            raw_csv = os.path.join(tmpdir, "ars620_canfd_test.raw.csv")
            mf4 = os.path.join(tmpdir, "ars620_canfd_test.mf4")
            data_headers = [f"data_{index:02d}" for index in range(64)]
            with open(raw_csv, "w") as handle:
                handle.write(
                    "timestamp_us,channel,can_id,is_extended,is_rtr,is_error,fd_flags,length,"
                    + ",".join(data_headers)
                    + "\n"
                )
                handle.write("1000000,1,291,0,0,0,5,3," + ",".join(["1", "171", "254"] + ["0"] * 61) + "\n")

            result = subprocess.run(
                ["python3", script, raw_csv, mf4],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=False,
            )
            if result.returncode == 2:
                self.assertIn("asammdf is required", result.stderr)
                return

            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(os.path.exists(mf4))
            self.assertGreater(os.path.getsize(mf4), 0)


if __name__ == "__main__":
    unittest.main()

