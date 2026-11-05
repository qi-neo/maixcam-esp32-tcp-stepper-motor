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


class FakeConn:
    """用可控的 recv 序列模拟对端，便于测超时 / 半包 / 超长行而不占端口。

    chunks 里每个元素是 bytes（一次 recv 返回的内容）；None 表示这次 recv 超时。
    """

    def __init__(self, chunks):
        self._chunks = list(chunks)
        self.closed = False

    def settimeout(self, _t):
        pass

    def recv(self, _n):
        if not self._chunks:
            raise ConnectionError("peer closed")
        nxt = self._chunks.pop(0)
        if nxt is None:
            raise socket.timeout("timeout")
        return nxt

    def close(self):
        self.closed = True


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


class MoreBuildCommandTests(unittest.TestCase):
    def test_checksum_on_appends_sum_field(self):
        with mock.patch.object(srv, "USE_CHECKSUM", True):
            frame = srv.build_command(1, 2, 3, 4)
            parts = frame.decode().strip().split(",")
            self.assertEqual(len(parts), 5, "启用校验和必须是 5 字段")
            self.assertEqual(int(parts[4]), (1 + 2 + 3 + 4) % 256)

    def test_checksum_off_is_four_fields(self):
        frame = srv.build_command(5, 6, 2, 9)
        self.assertEqual(len(frame.decode().strip().split(",")), 4)

    def test_seq_wrap_boundary(self):
        # SEQ_MOD-1 应原样作为序号，回绕到该值
        frame = srv.build_command(0, 0, 1, srv.SEQ_MOD - 1)
        self.assertEqual(frame, f"0,0,1,{srv.SEQ_MOD - 1}\n".encode())

    def test_negative_x_rejected(self):
        with self.assertRaises(ValueError):
            srv.build_command(-1, 0, 1, 0)

    def test_checksum_value_matches_ino_formula(self):
        # 与 ESP32 端 sum=(x+y+cat+seq)%256 完全一致
        with mock.patch.object(srv, "USE_CHECKSUM", True):
            for x, y, cat, seq in ((0, 0, 1, 0), (10, 20, 4, 250), (255, 255, 1, 255)):
                frame = srv.build_command(x, y, cat, seq).decode().strip()
                self.assertEqual(int(frame.split(",")[4]), (x + y + cat + seq) % 256)


class RobustLineBufferTests(unittest.TestCase):
    def test_timeout_raises_socket_timeout(self):
        conn = FakeConn([None])
        with self.assertRaises(socket.timeout):
            srv.read_line(conn, b"", 0.1)

    def test_partial_across_recv_then_complete(self):
        conn = FakeConn([b"ACK:", b"DONE:1:10:20:3\n", b"extra\n"])
        line, buf = srv.read_line(conn, b"", 0.5)
        self.assertEqual(line, "ACK:DONE:1:10:20:3")
        line2, _ = srv.read_line(conn, buf, 0.5)
        self.assertEqual(line2, "extra")

    def test_exact_boundary_not_rejected(self):
        # 恰好等于 MAX_LINE_BYTES 的行不应触发超长拒绝（只有 > 才拒绝）
        conn = FakeConn([b"x" * srv.MAX_LINE_BYTES + b"\n"])
        line, _ = srv.read_line(conn, b"", 0.5)
        self.assertEqual(len(line), srv.MAX_LINE_BYTES)

    def test_oversized_single_recv_rejected(self):
        conn = FakeConn([b"x" * (srv.MAX_LINE_BYTES + 1)])
        with self.assertRaises(ConnectionError):
            srv.read_line(conn, b"", 0.5)

    def test_oversized_accumulated_across_recv_rejected(self):
        # 分多次 recv 累计超过上限也应被拦下
        conn = FakeConn([b"a" * 100, b"b" * 100, b"c" * 100])
        with self.assertRaises(ConnectionError):
            srv.read_line(conn, b"", 0.5)


class MoreAuthTests(unittest.TestCase):
    def test_empty_token_default_allows(self):
        # 默认 AUTH_TOKEN 为空：不认证也能连（保持原行为）
        with mock.patch.object(srv, "AUTH_TOKEN", ""):
            ok, _ = srv.authenticate(FakeConn([]), b"")
            self.assertTrue(ok)

    def test_wrong_token_rejected(self):
        conn = FakeConn([b"AUTH nope\n"])
        with mock.patch.object(srv, "AUTH_TOKEN", "secret"):
            ok, _ = srv.authenticate(conn, b"")
            self.assertFalse(ok)

    def test_correct_token_trims_newline(self):
        # 首行带换行也能正确匹配（read_line 已 strip）
        conn = FakeConn([b"AUTH secret\n"])
        with mock.patch.object(srv, "AUTH_TOKEN", "secret"):
            ok, _ = srv.authenticate(conn, b"")
            self.assertTrue(ok)


if __name__ == "__main__":
    unittest.main()
