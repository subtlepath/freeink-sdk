# Browser simulator and e-ink lab

The static lab runs **real Tinta and lila C++ sources** compiled to WebAssembly
against FreeInk's simulator HAL. Firmware tasks and the virtual clock run on
pthread workers. Frames come from the existing SPI panel model, not a JavaScript
recreation of either application.

X3, X4 Classic and X4 Pro have separate builds, panel sizes and hardware
controls. Pro exposes its digitizer and capacitive Home key. Reset restarts the
firmware; pointer/keyboard holds reach electrical inputs. Losing focus releases
held inputs. Files imported onto the virtual SD card stay in the browser session.

The optional **Firmware image** mode runs the existing RISC-V/Xtensa CPU
emulator in WASM. Its peripheral compatibility is experimental; arbitrary
production ESP-IDF images may not boot. Use source builds for the product demos.
Radio, on-device network services and OTA report unavailable/error states.
S3 demos model the SSD1677 panel revision and seed its calibration byte.
Vector font/PSRAM behavior and embedded heap limits are not reproduced.

## Build

Activate Emscripten **4.0.23** and make the sibling source trees available:

```sh
pnpm install --dir tools/simulator/web
pnpm --dir tools/simulator/web vendor
sh ../spanish/tools/sim/build.sh X3
python3 tools/simulator/web/build.py
python3 tools/simulator/web/build-reader.py --device X3
python3 tools/simulator/web/build-reader.py --device X4CLASSIC
python3 tools/simulator/web/build-reader.py --device X4PRO
sh tools/simulator/web/build-emulator.sh
```

The Tinta fixture must exist at `../spanish/build/sim/course-sim.pack`.
`--reader`, `--spanish` and `--compiler` accept alternate paths. The reader uses
its own FreeInk SDK checkout and PlatformIO dependency sources from
`.pio/libdeps/default` (or `gh_release`). Build from a stable source snapshot if
the sibling tree is being edited. Reader objects track flags and header
dependencies. Generated WASM, content, images and caches are ignored here.

## Physical firmware and Web Serial

Compile installable images with PlatformIO, separately from WASM. Reader release
targets are `gh_release`, `x4c-gh_release`, `x4pro-gh_release`; development
`default`, `x4c`, `x4pro` outputs are also accepted. Tinta images come from its
`dist/manifest.json` and sparse full-flash HEX files.

```sh
python3 tools/simulator/web/prepare.py
# Cached PIO builds may omit firmware.factory.bin; use the framework's OTA data:
python3 tools/simulator/web/prepare.py --ota-data /path/to/framework-arduinoespressif32/tools/partitions/boot_app0.bin
```

`--reader /path/to/snapshot` and `--reader-builds` select isolated builds.
Packaging checks chip IDs, HEX checksums and erase boundaries. Factory NVS at
`0x9000..0xe000` is preserved. Regions have SHA-256 filenames; manifests link
matching source snapshots, build recipes and dependency notices. Source archives
exclude personal books, hardware backups and generated/cache files. The demo
EPUB contains only original text written for this project.

Initial X3 physical firmware uses a `web-x3-release` environment extending the
reader's `base`, with X3/X4 defines and version `0.1.0-web`. Its standard Arduino
core avoids a broken local tuned-core SCons build. S3 images use normal release
profiles. All packaged images are labeled development builds.

The installer uses esptool-js 0.7.0. A user chooses the USB serial port, confirms
the exact physical model and acknowledges a backup. The connected chip must
match. Every download passes length/SHA-256 checks before writing. Device MD5
verification runs before reset; flash settings remain intact and whole-chip
erase is disabled. Classic and Pro share a chip, so exact model selection
matters. No hardware was flashed during development; test backed-up devices
before promoting these builds.

## Preview and integrate into sp2

```sh
python3 tools/simulator/web/serve.py --port 8766
# http://127.0.0.1:8766/
node --test tools/simulator/web/test/*.test.mjs
python3 -m unittest discover -s tools/simulator/web/test -p 'test_*.py'
sh tools/simulator/test/run-panel.sh
sh tools/simulator/test/run-rtos.sh
FSIM_TEST_DEVICES=X3,X4CLASSIC,X4PRO node tools/simulator/web/test/browser.mjs
FSIM_TEST_APP=lila FSIM_TEST_DEVICES=X3,X4CLASSIC,X4PRO node tools/simulator/web/test/browser.mjs
node tools/simulator/web/test/flows.mjs
FSIM_TEST_URL=http://127.0.0.1:8766/ FSIM_TEST_DEVICES=X3,X4CLASSIC,X4PRO node tools/simulator/web/test/reader.mjs
python3 tools/simulator/web/integrate.py --stage tools/simulator/web/staging/site
python3 tools/simulator/web/serve.py --directory tools/simulator/web/staging/site --port 8767
# Review http://127.0.0.1:8767/ and /e-ink/, then apply:
python3 tools/simulator/web/integrate.py --site ../sp2/site
```

`FSIM_TEST_URL` selects another hosted/staged lab. Browser checks use desktop
Chrome on macOS, or Playwright Chromium on other platforms.
`PLAYWRIGHT_CHROMIUM_EXECUTABLE` overrides the executable. Flow tests capture
actual display images for the homepage feature.

Integration copies the lab, adds a prominent homepage R&D section and nav link,
updates the sitemap and extends host headers. The preview reads `_headers`,
including CSP, for realistic tests. On local HTTP it omits HSTS and CSP's
`upgrade-insecure-requests` so styles, scripts and links stay on the preview's
HTTP port; the production `_headers` file retains its HTTPS policy.
Hosting needs WASM MIME types,
`wasm-unsafe-eval`, same-origin workers/fetches, and COOP/COEP on the lab.
Serial is allowed for this origin. Cloudflare combines matching headers, so the
integrator extends the existing CSP rather than adding another restrictive one.
The resulting sp2 directory is ready for its existing static hosting;
**integration does not deploy**.

## Hardware references

Controls follow [Xteink's user guides](https://www.xteink.com/pages/user-guide):

- [X3](https://cdn.shopify.com/s/files/1/0759/9344/8689/files/X3_User_Guide_0914.pdf?v=1789369430)
- [Classic](https://cdn.shopify.com/s/files/1/0759/9344/8689/files/X4_Classic_User_Guide_0914.pdf?v=1789369505)
- [Pro](https://cdn.shopify.com/s/files/1/0759/9344/8689/files/X4_Pro_User_Guide_0818.pdf?v=1787034606)

Manufacturer case dimensions are X3 63.7 × 97.6 mm, Classic 69 × 114 mm and Pro
69 × 111 mm. Button centers are normalized from diagrams, not mechanical CAD
measurements. Button ABI indices match the SDK profiles; display and digitizer
coordinates use inverse rotations.
