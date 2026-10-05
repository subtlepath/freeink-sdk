#!/usr/bin/env python3
"""Local static preview with the headers needed for threaded WebAssembly."""
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
import argparse
import functools
import fnmatch


def preview_headers(directory, path=""):
    """Keep the site's isolation and CSP rules usable on this HTTP preview."""
    headers = {"Cross-Origin-Opener-Policy": "same-origin", "Cross-Origin-Embedder-Policy": "require-corp", "Cross-Origin-Resource-Policy": "same-origin", "Permissions-Policy": "serial=(self)"}
    rules = Path(directory) / "_headers"
    if rules.is_file():
        pattern = ""
        for line in rules.read_text().splitlines():
            if not line.strip() or line.lstrip().startswith("#"):
                continue
            if not line.startswith((" ", "\t")):
                pattern = line.strip()
            elif fnmatch.fnmatch(path.split("?", 1)[0], pattern):
                if line.strip().startswith("! "):
                    headers.pop(line.strip()[2:], None)
                elif ":" in line:
                    name, value = line.strip().split(":", 1)
                    headers[name] = value.strip()

    # The preview listens on HTTP only. Production HTTPS policy would upgrade
    # its styles, scripts and navigation to an unavailable TLS endpoint.
    for name in list(headers):
        if name.lower() == "strict-transport-security":
            del headers[name]
        elif name.lower() == "content-security-policy":
            headers[name] = "; ".join(
                directive.strip() for directive in headers[name].split(";")
                if directive.strip() and directive.strip().split()[0].lower() != "upgrade-insecure-requests"
            )
    return headers


class Handler(SimpleHTTPRequestHandler):
    extensions_map = {**SimpleHTTPRequestHandler.extensions_map, ".mjs": "text/javascript", ".wasm": "application/wasm"}
    def end_headers(self):
        # Malformed requests (including TLS sent to this HTTP port) can reach
        # send_error before parse_request has initialized self.path.
        headers = preview_headers(self.directory, getattr(self, "path", ""))
        for name, value in headers.items():
            if name != 'Content-Type': self.send_header(name, value)
        super().end_headers()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", default=str(Path(__file__).parent / "public"))
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    ThreadingHTTPServer(("127.0.0.1", args.port), functools.partial(Handler, directory=args.directory)).serve_forever()
