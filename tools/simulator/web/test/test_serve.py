"""Regression checks for local HTTP preview headers and malformed requests."""
import importlib.util
import io
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("preview", Path(__file__).parents[1] / "serve.py")
preview = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preview)


class FakeSocket:
    def __init__(self, request):
        self.request = io.BytesIO(request)
        self.response = bytearray()

    def makefile(self, *args, **kwargs):
        return self.request

    def sendall(self, data):
        self.response.extend(data)


class QuietHandler(preview.Handler):
    def log_message(self, *args):
        pass


class PreviewTests(unittest.TestCase):
    def test_local_http_keeps_csp_without_https_upgrades(self):
        with tempfile.TemporaryDirectory() as directory:
            rules = Path(directory) / "_headers"
            rules.write_text("/*\n  Content-Security-Policy: default-src 'none'; script-src 'self' 'wasm-unsafe-eval'; upgrade-insecure-requests\n  Strict-Transport-Security: max-age=31536000\n/e-ink/*\n  Cache-Control: no-cache\n")
            original = rules.read_text()
            headers = preview.preview_headers(directory, "/e-ink/?app=lila")
            self.assertEqual(headers["Content-Security-Policy"], "default-src 'none'; script-src 'self' 'wasm-unsafe-eval'")
            self.assertNotIn("Strict-Transport-Security", headers)
            self.assertEqual(headers["Cross-Origin-Embedder-Policy"], "require-corp")
            self.assertEqual(headers["Cache-Control"], "no-cache")
            self.assertEqual(rules.read_text(), original)

    def test_tls_request_to_http_port_does_not_crash_error_handler(self):
        with tempfile.TemporaryDirectory() as directory:
            (Path(directory) / "_headers").write_text("/*\n  X-Content-Type-Options: nosniff\n")
            socket = FakeSocket(b"\x16\x03\x01\x02\r\n")
            QuietHandler(socket, ("127.0.0.1", 12345), None, directory=directory)
            self.assertIn(b"Bad request syntax", socket.response)


if __name__ == "__main__":
    unittest.main()
