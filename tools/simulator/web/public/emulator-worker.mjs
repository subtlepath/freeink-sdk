let runtime, timer, previous = -1, bootPowerReleased = false;
const send = (type, detail = {}) => postMessage({ type, ...detail });
self.onmessage = async ({ data }) => {
  try {
    if (data.type === 'boot') {
      const { default: create } = await import('./runtime/emulator.mjs');
      runtime = await create();
      runtime.FS.mkdirTree('/sd');
      runtime.FS.mkdirTree('/rom');
      for (const chip of ['esp32c3', 'esp32s3']) {
        const response = await fetch(`./runtime/rom/${chip}.romsyms`);
        if (!response.ok) throw new Error('Missing ROM symbol map');
        runtime.FS.writeFile(`/rom/${chip}.romsyms`, await response.text());
      }
      runtime.FS.writeFile('/firmware.bin', new Uint8Array(data.bytes));
      if (!runtime.ccall('web_load', 'number', ['string', 'string'], ['/firmware.bin', data.device])) {
        throw new Error(runtime.ccall('web_error', 'string', [], []));
      }
      send('ready');
      timer = setInterval(() => {
        const running = runtime._web_run(100000);
        // Boot power hold is measured in guest time, not download/host time.
        if (!bootPowerReleased && runtime._web_time_ms() > 1800) { runtime._web_button(6, 0); bootPowerReleased = true; }
        const sequence = runtime._web_sequence();
        if (sequence !== previous) {
          previous = sequence;
          const ptr = runtime._web_frame(), width = runtime._web_width(), height = runtime._web_height();
          const pixels = runtime.HEAPU8.slice(ptr, ptr + width * height);
          postMessage({ type: 'frame', pixels, width, height, sequence }, [pixels.buffer]);
        }
        const text = runtime.ccall('web_logs', 'string', [], []);
        if (text) { send('log', { text }); runtime._web_clear_logs(); }
        if (!running) { clearInterval(timer); send('error', { message: runtime.ccall('web_error', 'string', [], []) }); }
      }, 4);
    } else if (data.type === 'button') runtime?._web_button(data.index, Number(data.down));
    else if (data.type === 'touch') runtime?._web_touch(data.x, data.y, Number(data.down));
  } catch (error) { clearInterval(timer); send('error', { message: error.message }); }
};
