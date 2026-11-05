#!/usr/bin/env python3
"""协议层离线单测：不需要 ESP32、不需要 Maixcam、不需要网络。

运行：python3 -m unittest discover -s tests -v
"""
import socket
import sys
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import maixcam_tcp_server as srv  # noqa: E402


class BuildCommandTests(unittest.TestCase):
    def test_basic_frame(self):
        self.assertEqual(srv.build_command(10, 20, 2, 7), b"10,20,2,7\n")

    def test_seq_wraps_at_seq_mod(self):
        self.assertEqual(srv.build_command(0, 0, 1, srv.SEQ_MOD + 1), b"0,0,1,1\n")

    def test_rejects_non_integer_fields(self):
        for bad in (True, 1.5, "1", None):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                srv.build_command(bad, 0, 1, 0)

    def test_rejects_out_of_range(self):
        with self.assertRaises(ValueError):
            srv.build_command(-1, 0, 1, 0)
        with self.assertRaises(ValueError):
            srv.build_command(srv.MAX_PIXEL_X + 1, 0, 1, 0)
        with self.assertRaises(ValueError):
            srv.build_command(0, 0, 5, 0)
        with self.assertRaises(ValueError):
            srv.build_command(0, srv.MAX_PIXEL_Y + 1, 1, 0)

    def test_checksum_default_is_off(self):
        self.assertFalse(srv.USE_CHECKSUM)
        self.assertEqual(srv.build_command(1, 2, 3, 4), b"1,2,3,4\n")


class AckTests(unittest.TestCase):
    def test_parse_ack(self):
        self.assertEqual(
            srv.parse_ack("ACK:DONE:2:100:200:7"),
            {"status": "DONE", "cat": 2, "pos1": 100, "pos2": 200, "seq": 7},
        )

    def test_control_ack_uses_negative_seq(self):
        ack = srv.parse_ack("ACK:ESTOP:0:0:0:-1")
        self.assertEqual(ack["seq"], srv.CONTROL_SEQ)

    def test_rejects_malformed_ack(self):
        for line in ("", "hello", "ACK:DONE:2:100:200", "ACK:DONE:x:100:200:7", "NOPE:DONE:2:100:200:7"):
            with self.subTest(line=line):
                self.assertIsNone(srv.parse_ack(line))

    def test_ack_matches_only_current_seq(self):
        ack = {"seq": 7}
        self.assertTrue(srv.ack_matches(ack, 7))
        self.assertFalse(srv.ack_matches(ack, 8))
        self.assertFalse(srv.ack_matches(None, 7))


class LineBufferTests(unittest.TestCase):
    def setUp(self):
        self.a, self.b = socket.socketpair()
        self.a.settimeout(0.5)
        self.b.settimeout(0.5)

    def tearDown(self):
        self.a.close()
        self.b.close()

    def test_two_lines_in_one_read(self):
        self.a.sendall(b"first\nsecond\n")
        line, buf = srv.read_line(self.b, b"", 0.5)
        self.assertEqual(line, "first")
        line2, buf2 = srv.read_line(self.b, buf, 0.5)
        self.assertEqual(line2, "second")
        self.assertEqual(buf2, b"")

    def test_half_line_is_kept(self):
        self.a.sendall(b"ACK:DONE:1")
        self.a.sendall(b":10:20:3\n")
        line, buf = srv.read_line(self.b, b"", 0.5)
        self.assertEqual(line, "ACK:DONE:1:10:20:3")
        self.assertEqual(buf, b"")

    def test_unbounded_line_is_rejected(self):
        self.a.sendall(b"x" * (srv.MAX_LINE_BYTES + 50))
        with self.assertRaises(ConnectionError):
            srv.read_line(self.b, b"", 0.5)

    def test_peer_close_returns_none(self):
        self.a.sendall(b"partial")
        self.a.close()
        line, buf = srv.read_line(self.b, b"", 0.5)
        self.assertIsNone(line)
        self.assertEqual(buf, b"partial")


class AuthTests(unittest.TestCase):
    def setUp(self):
        self.a, self.b = socket.socketpair()
        self.a.settimeout(0.5)
        self.b.settimeout(0.5)

    def tearDown(self):
        self.a.close()
        self.b.close()

    def test_no_token_allows_connection(self):
        with mock.patch.object(srv, "AUTH_TOKEN", ""):
            ok, _ = srv.authenticate(self.b, b"")
            self.assertTrue(ok)

    def test_correct_token(self):
        self.a.sendall(b"AUTH s3cr3t\n")
        with mock.patch.object(srv, "AUTH_TOKEN", "s3cr3t"):
            ok, _ = srv.authenticate(self.b, b"")
            self.assertTrue(ok)

    def test_wrong_token_rejected(self):
        self.a.sendall(b"AUTH wrong\n")
        with mock.patch.object(srv, "AUTH_TOKEN", "s3cr3t"):
            ok, _ = srv.authenticate(self.b, b"")
            self.assertFalse(ok)


if __name__ == "__main__":
    unittest.main()
