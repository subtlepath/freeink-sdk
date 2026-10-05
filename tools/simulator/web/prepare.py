#!/usr/bin/env python3
"""Package local firmware and demo content as immutable, hashed static assets."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import zipfile

HERE = Path(__file__).resolve().parent
SDK = HERE.parents[2]

def hex_regions(text):
    """Read ordered Intel HEX without filling holes (especially factory NVS)."""
    regions = []
    base = 0
    eof = False
    for line in text.splitlines():
        if not line.strip(): continue
        if eof or not line.startswith(':'): raise ValueError('Invalid Intel HEX record')
        data = bytes.fromhex(line[1:])
        if len(data)<5 or len(data)!=data[0]+5 or sum(data)&255: raise ValueError('Intel HEX length/checksum failed')
        address = base + int.from_bytes(data[1:3], 'big')
        kind = data[3]
        payload = data[4:-1]
        if kind == 0:
            if not payload: continue
            if regions and address < regions[-1][0]+len(regions[-1][1]): raise ValueError('HEX regions overlap or are unordered')
            if regions and address == regions[-1][0]+len(regions[-1][1]): regions[-1][1].extend(payload)
            else: regions.append((address, bytearray(payload)))
        elif kind == 1:
            if payload: raise ValueError('Invalid EOF record')
            eof = True
        elif kind == 4:
            if len(payload)!=2: raise ValueError('Invalid extended address')
            base = int.from_bytes(payload, 'big') << 16
        elif kind == 2:
            if len(payload)!=2: raise ValueError('Invalid segment address')
            base = int.from_bytes(payload, 'big') << 4
        elif kind not in (3,5): raise ValueError('Unsupported HEX record')
    if not eof or not regions: raise ValueError('Truncated or empty HEX file')
    return [(address, bytes(data)) for address,data in regions]

def package(out, app, device, title, version, regions, draft=True):
    plan = dict(app=app, device=device, title=title, version=version, draft=draft, chip='esp32c3' if device=='X3' else 'esp32s3', preserve=[dict(start=0x9000,end=0xe000)], files=[])
    chip_id = 5 if device == 'X3' else 9
    def read_at(address, count):
        for start, data in regions:
            if start <= address and address + count <= start + len(data): return data[address-start:address-start+count]
        raise ValueError(f'Missing firmware header at {address:#x}')
    for address in [0, 0x10000]:
        header = read_at(address, 24)
        if header[0] != 0xe9 or int.from_bytes(header[12:14], 'little') != chip_id:
            raise ValueError(f'{device}: bootloader/application targets the wrong chip')
    previous = 0
    for address,data in regions:
        start = address//4096*4096
        end = (address+len(data)+4095)//4096*4096
        if start<previous or start<0xe000 and end>0x9000 or end>0x1000000: raise ValueError('Flash plan touches protected data or overlaps erase sectors')
        digest = hashlib.sha256(data).hexdigest()
        name = f'{app}-{device.lower()}-{address:x}-{digest[:12]}.bin'
        (out/name).write_bytes(data)
        plan['files'].append(dict(address=address, size=len(data), sha256=digest, url=name))
        previous = end
    return plan

def source_archive(out, root, app):
    """Ship the actual source snapshot and build recipes next to binaries."""
    temporary = out/f'{app}-source.zip'
    excluded = {'.git', '.pio', '.cache', '.dummy', 'node_modules', '__pycache__', 'build', 'dist', '.aws', '.codex', '.pnpm-store', 'managed_components', 'staging', 'releases', 'runtime'}
    source_dirs = {'src','lib','libs','include','scripts','test','tests','freeink-sdk','assets','tools','content','course','courses'}
    root_files = {'LICENSE','README.md','platformio.ini','partitions.csv','library.json','CMakeLists.txt'}
    with zipfile.ZipFile(temporary, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(root.rglob('*')):
            relative = path.relative_to(root)
            if relative.parts[0] not in source_dirs and str(relative) not in root_files: continue
            if not path.is_file() or any(p in excluded for p in relative.parts) or path.suffix in {'.bin', '.elf', '.o', '.a', '.epub', '.pdf', '.zip', '.xz'}: continue
            if relative.parts[:2] == ('lib','Tinta'): continue
            archive.write(path, str(Path(app)/relative))
        deps = root/'.pio/libdeps'
        targets = sorted(deps.iterdir()) if deps.is_dir() else []
        if targets:
            for library in sorted(targets[0].iterdir()):
                if not library.is_dir() or library.is_symlink(): continue
                for path in sorted(library.rglob('*')):
                    if path.is_file() and not any(p in excluded for p in path.parts) and path.suffix not in {'.bin','.elf','.o','.a','.epub','.zip'}:
                        archive.write(path, str(Path(app)/'dependencies'/library.name/path.relative_to(library)))
    digest = hashlib.sha256(temporary.read_bytes()).hexdigest()[:12]
    target = out/f'{app}-source-{digest}.zip'
    temporary.replace(target)
    return target.name

def demo_book(path):
    # Original text, generated locally; no copyrighted user EPUBs are copied.
    chapter = '''<html xmlns="http://www.w3.org/1999/xhtml"><head><title>A little book of attention</title></head><body><h1>A little book of attention</h1><p>Reading makes a small room in the day. A page, a pause, a thought worth keeping.</p>'''
    for i in range(1,70): chapter += f'<h2>A moment, {i}</h2><p>The best tools give us somewhere to put our attention. Outside the window the trees move in the wind. Here, a few words stay still long enough to become an idea.</p><p>Try turning a page. Change the type size. Let the screen hold its place while you find yours.</p>'
    chapter += '</body></html>'
    with zipfile.ZipFile(path,'w') as book:
        book.writestr('mimetype','application/epub+zip',compress_type=zipfile.ZIP_STORED)
        book.writestr('META-INF/container.xml','<?xml version="1.0"?><container version="1.0" xmlns="urn:oasis:names:tc:opendocument:xmlns:container"><rootfiles><rootfile full-path="OEBPS/content.opf" media-type="application/oebps-package+xml"/></rootfiles></container>')
        book.writestr('OEBPS/content.opf','''<?xml version="1.0"?><package xmlns="http://www.idpf.org/2007/opf" version="2.0" unique-identifier="id"><metadata xmlns:dc="http://purl.org/dc/elements/1.1/"><dc:identifier id="id">freeink-web-demo</dc:identifier><dc:title>A little book of attention</dc:title><dc:creator>Subtle Path</dc:creator><dc:language>en</dc:language></metadata><manifest><item id="chapter" href="chapter.xhtml" media-type="application/xhtml+xml"/><item id="ncx" href="toc.ncx" media-type="application/x-dtbncx+xml"/></manifest><spine toc="ncx"><itemref idref="chapter"/></spine></package>''')
        book.writestr('OEBPS/chapter.xhtml',chapter,compress_type=zipfile.ZIP_DEFLATED)
        book.writestr('OEBPS/toc.ncx','''<?xml version="1.0"?><ncx xmlns="http://www.daisy.org/z3986/2005/ncx/" version="2005-1"><head><meta name="dtb:uid" content="freeink-web-demo"/></head><docTitle><text>A little book of attention</text></docTitle><navMap><navPoint id="one" playOrder="1"><navLabel><text>A little book of attention</text></navLabel><content src="chapter.xhtml"/></navPoint></navMap></ncx>''')

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spanish',type=Path,default=SDK.parent/'spanish')
    parser.add_argument('--reader',type=Path,default=SDK.parent/'crosspoint-reader')
    parser.add_argument('--reader-builds',type=Path, help='An isolated PlatformIO build directory')
    parser.add_argument('--ota-data',type=Path, help='Arduino tools/partitions/boot_app0.bin, if the build cache omitted factory images')
    parser.add_argument('--out',type=Path,default=HERE/'public')
    args = parser.parse_args()
    releases = args.out/'releases'; releases.mkdir(parents=True,exist_ok=True)
    content = args.out/'content'; content.mkdir(parents=True,exist_ok=True)
    demo_book(content/'demo.epub')
    plans = []
    dist = args.spanish/'dist'
    manifest = json.loads((dist/'manifest.json').read_text())
    for device,build in [('X3','x3-x4'),('X4CLASSIC','x4-classic'),('X4PRO','x4-pro')]:
        image = next(image for image in manifest['images'] if image['build']==build)
        regions = hex_regions((dist/image['full_flash']).read_text())
        plans.append(package(releases,'tinta',device,'Tinta',manifest['version'],regions,manifest.get('draft',True)))
    builds = args.reader_builds or args.reader/'.pio/build'
    for device,targets in [('X3',['web-x3-release','gh_release','default']),('X4CLASSIC',['x4c-gh_release','x4c']),('X4PRO',['x4pro-gh_release','x4pro'])]:
        build = next((builds/target for target in targets if (builds/target/'firmware.bin').is_file()), builds/targets[0])
        if not (build/'firmware.bin').is_file(): continue
        # Exact source build parts: no merged .bin that would erase NVS gaps.
        boot = build/'bootloader.bin'; partitions = build/'partitions.bin'
        factory = build/'firmware.factory.bin'
        if not all(p.is_file() for p in [boot,partitions]): continue
        if factory.is_file(): otadata = factory.read_bytes()[0xe000:0x10000]
        elif args.ota_data: otadata = args.ota_data.read_bytes()
        else:
            print(f'Skipping {device}: cached build omitted factory image; pass --ota-data boot_app0.bin')
            continue
        if len(otadata) != 8192: raise ValueError('Initial OTA selector must be exactly 8192 bytes')
        version = '0.1.0-web' if build.name == 'web-x3-release' else '0.1.0-local'
        plans.append(package(releases,'lila',device,'lila',version,[(0,boot.read_bytes()),(0x8000,partitions.read_bytes()),(0xe000,otadata+(build/'firmware.bin').read_bytes())]))
    for app,root in [('tinta',args.spanish),('lila',args.reader)]:
        source = source_archive(releases,root,app)
        for plan in plans:
            if plan['app'] == app: plan['source_url'] = source
    (releases/'index.json').write_text(json.dumps(plans,indent=2)+'\n')
    licenses = releases/'licenses'; licenses.mkdir(exist_ok=True)
    license_source = dist/'licenses' if (dist/'licenses').is_dir() else args.spanish/'tools/release/licenses'
    shutil.copytree(license_source,licenses,dirs_exist_ok=True)
    shutil.copyfile(args.reader/'LICENSE',licenses/'lila-LICENSE')
    shutil.copyfile(SDK/'LICENSE',licenses/'freeink-LICENSE')
    for name in ['GUST-FONT-LICENSE.txt','lppl.txt']:
        shutil.copyfile(args.spanish/'assets/fonts/outline'/name,licenses/name)
    for path in sorted((args.reader/'lib').rglob('*')):
        if path.is_file() and path.name in {'LICENSE','LICENSE.txt','LICENSE.md','OFL.txt','COPYING'}:
            shutil.copyfile(path,licenses/('-'.join(path.relative_to(args.reader/'lib').parts)))
    dep_targets = sorted((args.reader/'.pio/libdeps').iterdir())
    if dep_targets:
        for path in sorted(dep_targets[0].rglob('*')):
            if path.is_file() and path.name in {'LICENSE','LICENSE.txt','LICENSE.md','COPYING'}:
                shutil.copyfile(path,licenses/('-'.join(path.relative_to(dep_targets[0]).parts)))
    shutil.copyfile(HERE/'node_modules/esptool-js/LICENSE',licenses/'esptool-js-LICENSE')
    pako_license = (HERE/'node_modules/esptool-js').resolve().parent/'pako/LICENSE'
    if pako_license.is_file(): shutil.copyfile(pako_license,licenses/'pako-LICENSE')
    (licenses/'READER-NOTICE.txt').write_text('lila browser demo: MIT application, FreeInk SDK, ArduinoJson, PNGdec, JPEGDEC, QRCode, miniz, uzlib, expat and MiniBidi; bundled fonts carry their own notices. Browser installation uses esptool-js (Apache 2.0) and pako (MIT). Physical reader images also link the Arduino ESP32 core (LGPL 2.1 or later), ESP-IDF (Apache 2.0), FreeRTOS (MIT), TinyUSB (MIT), and Arduino-wolfSSL (GPL 2). The matching application, SDK, dependency sources and build recipes are provided in the lila-source archive alongside the images. The application source remains MIT; distributing the complete wolfSSL-linked firmware is subject to GPL 2. Rebuild with PlatformIO using the included platformio.ini and its pinned dependencies. X3 web-x3-release uses the standard prebuilt Arduino core. No warranty.\n')
    print(f'Packaged {len(plans)} exact-board firmware plans and an original demo EPUB.')
if __name__ == '__main__': main()
