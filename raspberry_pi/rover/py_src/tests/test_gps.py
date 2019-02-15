# -*- coding: utf-8 -*-
import unittest
import rover_module


class TestGPS(unittest.TestCase):
    def test_gps_reader_GPRMC(self):
        self.assertEqual(rover_module.gps_reader('$GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62'), [-37.86083333333333, 145.12266666666667])

    def test_gps_reader_GPGGA(self):
        self.assertEqual(rover_module.gps_reader('$GPGGA,085120.307,3541.1493,N,13945.3994,E,1,08,1.0,6.9,M,35.9,M,,0000*5E'), [35.68582166666667, 139.75665666666666])


if __name__ == "__main__":
    unittest.main()