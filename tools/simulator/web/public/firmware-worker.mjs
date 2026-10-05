let runtime, timer, previous = -1, paused = false, activeApp;
const send = (type, detail = {}) => postMessage({ type, ...detail });
function restart() {
  clearInterval(timer);
  const files = [];
  function walk(path) {
    for (const name of runtime.FS.readdir(path)) {
      if (name === '.' || name === '..') continue;
      const child = `${path}/${name}`;
      if (runtime.FS.isDir(runtime.FS.stat(child).mode)) walk(child);
      else files.push({ path: child, bytes: runtime.FS.readFile(child) });
    }
  }
  walk('/sd');
  send('restart', { files });
}

async function boot({ app, device, files = [] }) {
  activeApp = app;
  if (!self.crossOriginIsolated) throw new Error('The host must enable cross-origin isolation for the simulator.');
  const { default: create } = await import(`./runtime/${app}-${device.toLowerCase()}.mjs`);
  runtime = await create({ print: text => send('log', { text }), printErr: text => send('log', { text }) });
  runtime.FS.mkdirTree('/sd');
  if (app === 'tinta') {
    const response = await fetch('./runtime/course.pack');
    if (!response.ok) throw new Error('The demo course pack is unavailable.');
    runtime.FS.writeFile('/course.pack', new Uint8Array(await response.arrayBuffer()));
  } else {
    const response = await fetch('./content/demo.epub');
    if (!response.ok) throw new Error('The demo book is unavailable.');
    runtime.FS.writeFile('/sd/A little book of attention.epub', new Uint8Array(await response.arrayBuffer()));
  }
  for (const file of files) {
    if (!file.path.startsWith('/sd/') || file.path.split('/').includes('..')) throw new Error('Invalid virtual-card path');
    runtime.FS.mkdirTree(file.path.slice(0, file.path.lastIndexOf('/')));
    runtime.FS.writeFile(file.path, new Uint8Array(file.bytes));
  }
  runtime._web_start(Number(app === 'lila'));
  send('ready');
  timer = setInterval(() => {
    if (paused) return;
    const state = runtime._web_tick();
    const sequence = runtime._web_sequence();
    if (sequence !== previous) {
      previous = sequence;
      const ptr = runtime._web_frame();
      const width = runtime._web_width(), height = runtime._web_height();
      if (width > 0 && height > 0) {
        const pixels = runtime.HEAPU8.slice(ptr, ptr + width * height);
        postMessage({ type: 'frame', width, height, sequence, pixels }, [pixels.buffer]);
      }
    }
    const text = runtime.ccall('web_logs', 'string', [], []);
    if (text) send('log', { text });
    if (state === 2) restart();
    else send('state', { asleep: state === 1 });
  }, 20);
}

self.onmessage = async ({ data }) => {
  try {
    if (data.type === 'boot') await boot(data);
    else if (data.type === 'reset') restart();
    else if (data.type === 'button') runtime?._web_button(data.index, Number(data.down));
    else if (data.type === 'touch') runtime?._web_touch(data.x, data.y, Number(data.down));
    else if (data.type === 'pause') paused = data.paused;
    else if (data.type === 'file') {
      // A file dropped onto the virtual card is only read inside the simulator.
      const name = data.name.replace(/[\\/\x00-\x1f]/g, '_').slice(0, 180);
      runtime.FS.writeFile(`/sd/${name}`, new Uint8Array(data.bytes));
      if (activeApp === 'lila') {
        // External card changes must invalidate the reader's persistent shelf.
        runtime.FS.mkdirTree('/sd/.crosspoint');
        runtime.FS.writeFile('/sd/.crosspoint/library.dirty', new Uint8Array([1]));
      }
      send('log', { text: `Added ${name} to the virtual SD card. Restart to rescan.` });
    }
  } catch (error) { clearInterval(timer); send('error', { message: error.message }); }
};
