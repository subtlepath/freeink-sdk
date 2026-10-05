#!/usr/bin/env python3
"""Build the sibling CrossPoint/lila source tree using the simulator HAL."""
import argparse
import concurrent.futures
import hashlib
import pathlib
import subprocess
import sys
import shlex
from build import HERE, SDK, SIM

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reader', type=pathlib.Path, default=SDK.parent / 'crosspoint-reader')
    parser.add_argument('--compiler', default='em++')
    parser.add_argument('--device', default='X3', choices=['X3','X4CLASSIC','X4PRO'])
    args = parser.parse_args()
    root = args.reader.resolve()
    sdk = root / 'freeink-sdk'
    out = HERE / 'public/runtime'
    objects = HERE / 'build' / args.device
    objects.mkdir(parents=True, exist_ok=True)
    compat = HERE / 'compat'
    includes = [compat, SDK.parent/'spanish/tools/sim/compat', out/'compat', SIM/'include', SIM/'platform', root/'src']
    includes += sorted((root/'lib').iterdir())
    includes += [root/'lib/uzlib/src', SDK/'libs/book/FreeInkBook/src', SDK/'libs/book/FreeInkBook/src/inflate']
    includes += [root/'lib/miniz/src']
    includes += sorted((sdk/'libs').glob('*/*/include'))
    deps = root / '.pio/libdeps/default'
    if not deps.is_dir(): deps = root / '.pio/libdeps/gh_release'
    includes += [deps/name/'src' for name in ['ArduinoJson','JPEGDEC','PNGdec','QRCode','WebSockets']]
    sources = [SIM/'core'/f'{name}.cpp' for name in ['Machine','Panel','I2cDevices','VirtualCard','Fat32Image','SdSpiCard','SdNativeCard','Abi']]
    sources += [SIM/'platform'/f'{name}.cpp' for name in ['sim_bridge','sim_rtos','sim_fs','sim_entry']]
    sources += [sdk/'libs/display/FreeInkDisplay/src'/name for name in ['FreeInkDisplay.cpp','bus/EpdBus.cpp','driver/Ssd1677Driver.cpp','driver/Uc8253X3Driver.cpp','driver/Uc8179Driver.cpp','driver/Uc8279Driver.cpp','driver/Uc8279X4Driver.cpp']]
    for name in ['InputManager','XteinkDetect','BatteryMonitor','PowerManager','FrontlightManager','Rtc','Imu','MemoryManager']:
        sources += sorted((sdk/f'libs/hardware/{name}/src').glob('*.cpp'))
    sources += [sdk/'libs/ui/FreeInkUI/src/FreeInkUI.cpp']
    sources += [sdk/'libs/font/FreeInkFont/src/FontAlloc.c']
    sources += sorted((sdk/'libs/network/NearbyTransfer/src').glob('*.cpp'))
    sources += sorted((root/'src').rglob('*.cpp'))
    for library in sorted((root/'lib').iterdir()):
        # The Spanish application may be checked out as an optional library;
        # it has its own WASM target and is not linked into the reader demo.
        if library.is_dir() and library.name != 'Tinta':
            sources += sorted(library.rglob('*.cpp')) + sorted(library.rglob('*.c'))
    sources += [deps/'JPEGDEC/src/JPEGDEC.cpp', deps/'PNGdec/src/PNGdec.cpp', deps/'QRCode/src/qrcode.c', HERE/'firmware.cpp']
    sources += sorted((deps/'PNGdec/src').glob('*.c'))
    sources = [s for s in sources if s.name != 'HalSystem.cpp' and s.parent != root/'src/network' and s.name != 'miniz.c']
    sources += [HERE/'reader-offline.cpp', SIM/'emu/Md5.cpp']
    flags = ['-O1','-pthread', '-DFSIM_WEB=1','-DFREEINK_CAP_USB_MSC=0','-DRTC_NOINIT_ATTR=', '-DEINK_DISPLAY_SINGLE_BUFFER_MODE=1', '-DUSE_UTF8_LONG_NAMES=1', '-DLOG_LEVEL=2', '-DXML_CONTEXT_BYTES=1024','-DXML_GE=0','-DMINIZ_NO_ZLIB_COMPATIBLE_NAMES=1', '-DCROSSPOINT_VERSION="0.1.0-web"','-DCROSSPOINT_UPSTREAM_VERSION="1.6.5"', f'-DFREEINK_DEVICE_{args.device}=1','-DFSIM_SOC_SPI_PERIPH_NUM='+('1' if args.device=='X3' else '2'), '-DFSIM_SOC_GPIO_PIN_COUNT='+('22' if args.device=='X3' else '48')]
    flags += [f'-I{p}' for p in includes if p.is_dir()]
    flags += ['-DARDUINOJSON_ENABLE_ARDUINO_STRING=1', '-DARDUINOJSON_ENABLE_ARDUINO_STREAM=1', '-DENABLE_SERIAL_LOG=1']
    def compile_one(source):
        obj = objects / (hashlib.sha256(str(source).encode()).hexdigest()[:16]+'.o')
        signature = hashlib.sha256(('\n'.join(flags)+str(source)).encode()).hexdigest()
        stamp = obj.with_suffix('.signature')
        dependency_file = obj.with_suffix('.d')
        if obj.exists() and stamp.exists() and dependency_file.exists() and stamp.read_text()==signature:
            dependencies = shlex.split(dependency_file.read_text().replace('\\\n', '').split(':', 1)[1])
            if all(pathlib.Path(p).exists() and pathlib.Path(p).stat().st_mtime < obj.stat().st_mtime for p in dependencies): return obj, ''
        compiler = args.compiler if source.suffix=='.cpp' else str(pathlib.Path(args.compiler).with_name('emcc'))
        command = [compiler, *flags, *(['-std=c++20', '-include', str(compat/'Esp.h')] if source.suffix=='.cpp' else []), '-MMD', '-MF', str(dependency_file), '-c', str(source), '-o', str(obj)]
        result = subprocess.run(command, capture_output=True, text=True)
        if result.returncode:
            return None, '\n'.join(line for line in result.stderr.splitlines() if 'error:' in line or 'fatal error:' in line)
        stamp.write_text(signature)
        return obj, ''
    with concurrent.futures.ThreadPoolExecutor(max_workers=6) as executor:
        results = list(executor.map(compile_one, sources))
    errors = [(str(source), error) for source,(obj,error) in zip(sources,results) if obj is None]
    for source, error in errors: print(source+'\n'+error+'\n', flush=True)
    print(f'{len(sources)-len(errors)}/{len(sources)} compiled', flush=True)
    if errors: sys.exit(1)
    subprocess.run([args.compiler, '-pthread', *[str(obj) for obj,_ in results], '-Wl,--error-limit=0', '--no-entry', '-sMODULARIZE=1', '-sEXPORT_ES6=1', '-sENVIRONMENT=web,worker,node', '-sALLOW_MEMORY_GROWTH=1', '-sINITIAL_MEMORY=67108864', '-sMAXIMUM_MEMORY=536870912', '-sPTHREAD_POOL_SIZE=8', '-sDEFAULT_PTHREAD_STACK_SIZE=2097152', "-sEXPORTED_RUNTIME_METHODS=['FS','ccall','UTF8ToString','HEAPU8']", '-o', str(out/f'lila-{args.device.lower()}.mjs')], check=True)
if __name__ == '__main__': main()
