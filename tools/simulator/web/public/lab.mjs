import { DEVICES, KEY_BUTTONS, availableButton, portraitPixels } from './devices.mjs';
import './flasher.mjs';

const $ = id => document.getElementById(id);
let worker, running = false, asleep = false, frame = 0, selectedImage, bootId = 0;
const held = new Map();
const cardFiles = new Map();
const cardStates = new Map();
const deviceId = () => document.querySelector('input[name=device]:checked').value;
const send = data => worker?.postMessage(data);
const status = text => { $('status').textContent = text; };
const appendLog = text => { $('console').textContent = ($('console').textContent + text + '\n').slice(-24000); };
function button(index, down, source) {
  if (!running) return;
  if (down) held.set(source, index); else held.delete(source);
  const pressed = [...held.values()].includes(index);
  send({ type: 'button', index, down: pressed });
  document.querySelector(`[data-index="${index}"]`)?.classList.toggle('pressed', pressed);
}
function releaseAll() {
  for (const index of new Set(held.values())) send({ type: 'button', index, down: false });
  held.clear();
  for (const el of document.querySelectorAll('.pressed')) el.classList.remove('pressed');
  send({ type: 'touch', x: 0, y: 0, down: false });
}
function attachKey(el, index) {
  el.addEventListener('pointerdown', event => { event.preventDefault(); el.setPointerCapture(event.pointerId); button(index, true, `pointer-${event.pointerId}`); });
  const release = event => button(index, false, `pointer-${event.pointerId}`);
  el.addEventListener('pointerup', release); el.addEventListener('pointercancel', release); el.addEventListener('lostpointercapture', release);
  // Native keyboard activation and assistive technologies dispatch click.
  el.addEventListener('click', event => { if (event.detail === 0) { button(index, true, 'activation'); setTimeout(() => button(index, false, 'activation'), 140); } });
}
function renderDevice() {
  releaseAll();
  const id = deviceId(), device = DEVICES[id];
  $('device').dataset.model = id;
  $('device').style.aspectRatio = `${device.width}/${device.height}`;
  $('hardware-spec').textContent = `${device.name} · ${device.screen.join(' × ')} · ${device.touch ? 'touch + physical keys' : 'physical keys'}`;
  $('hardware-buttons').replaceChildren();
  const reset = document.createElement('button');
  reset.className = 'reset-pinhole'; reset.setAttribute('aria-label', 'Reset pinhole');
  reset.style.left = `${device.reset}%`; reset.addEventListener('click', restart);
  $('hardware-buttons').append(reset);
  for (const key of device.buttons) {
    const el = document.createElement('button');
    el.className = 'hardware-key'; el.dataset.index = key.index; el.dataset.edge = key.edge; el.setAttribute('aria-label', key.label);
    el.style[['left', 'right'].includes(key.edge) ? 'top' : 'left'] = `${key.position}%`;
    const label = document.createElement('span'); label.textContent = key.label; el.append(label);
    attachKey(el, key.index); $('hardware-buttons').append(el);
  }
  $('home-key').hidden = !device.touch;
  $('screen').width = device.screen[0]; $('screen').height = device.screen[1];
  $('screen').getContext('2d').fillStyle = '#fbfbf4';
  $('screen').getContext('2d').fillRect(0, 0, ...device.screen);
  $('input-hint').textContent = device.touch ? 'Tap the screen, the two page keys, or Home. Up/down arrows turn pages. P is Power. Hold a key for a long press.' : 'Press the keys on the device. Arrow keys navigate; Enter confirms; Esc goes back. P is Power. Hold a key for a long press.';
}
function stop() { releaseAll(); worker?.terminate(); worker = undefined; running = false; asleep = false; frame = 0; }
async function start() {
  stop(); const id = ++bootId, app = $('app').value, device = deviceId();
  $('console').textContent = ''; status('Starting firmware…');
  if (app === 'image' && !selectedImage) { status('Choose a firmware .bin image first.'); return; }
  worker = new Worker(new URL(app === 'image' ? './emulator-worker.mjs' : './firmware-worker.mjs', import.meta.url), { type: 'module' });
  const current = worker;
  worker.onerror = event => { if (id === bootId) { running = false; status('The simulator stopped. See the firmware console.'); appendLog(event.message); } };
  worker.onmessage = ({ data }) => {
    if (id !== bootId) return;
    if (data.type === 'ready') { running = true; status('Firmware running · booting display'); }
    if (data.type === 'log') appendLog(data.text);
    if (data.type === 'frame') {
      const { rgba, width, height } = portraitPixels(data.pixels, data.width, data.height);
      $('screen').width = width; $('screen').height = height;
      $('screen').getContext('2d').putImageData(new ImageData(rgba, width, height), 0, 0);
      frame = data.sequence; if (frame) status(`${DEVICES[device].name} · live firmware · refresh ${frame}`);
    }
    if (data.type === 'state' && data.asleep !== asleep) { asleep = data.asleep; if (asleep) status('Device asleep · press Power to wake'); }
    if (data.type === 'restart') { if (data.files) cardStates.set(`${app}-${device}`, data.files); start(); }
    if (data.type === 'error') { running = false; releaseAll(); status(data.message); appendLog(data.message); }
  };
  if (app === 'image') { const bytes = await selectedImage.arrayBuffer(); if (id === bootId) current.postMessage({ type: 'boot', device, bytes }, [bytes]); }
  else current.postMessage({ type: 'boot', app, device, files: [...(cardStates.get(`${app}-${device}`) || []), ...[...cardFiles].map(([name, bytes]) => ({ path: `/sd/${name.replace(/[\\/\x00-\x1f]/g, '_').slice(0, 180)}`, bytes }))] });
}
function restart() { if (running && $('app').value !== 'image') send({ type: 'reset' }); else start(); }
$('start').addEventListener('click', restart); $('restart').addEventListener('click', restart);
for (const input of document.querySelectorAll('input[name=device]')) input.addEventListener('change', () => { stop(); renderDevice(); status('Device selected · start the demo'); });
$('app').addEventListener('change', () => {
  stop(); renderDevice(); const app = $('app').value;
  $('image-input-label').hidden = app !== 'image';
  $('card-file').closest('details').hidden = app === 'image';
  $('app-description').textContent = app === 'tinta' ? 'Learn a little Mexican Spanish every day. Lessons, spaced repetition, a dictionary and stories — all offline.' : app === 'lila' ? 'An EPUB and plain-text reader, exploring typography and reading rhythm. The actual CrossPoint fork, compiled for the browser.' : 'Run ESP32-C3 or ESP32-S3 instructions against virtual hardware. Compatibility depends on the existing CPU and peripheral models; boot failures appear in the console.';
  status('Software selected · start the demo');
});
$('image-input').addEventListener('change', event => { selectedImage = event.target.files[0]; });
window.addEventListener('blur', releaseAll);
document.addEventListener('visibilitychange', () => { if (document.hidden) releaseAll(); });
document.addEventListener('keydown', event => {
  if (event.target.closest('input,select,textarea,button,summary,a') || event.ctrlKey || event.metaKey || event.altKey) return;
  const index = KEY_BUTTONS[event.key];
  if (index !== undefined && availableButton(deviceId(), index)) { event.preventDefault(); if (!event.repeat) button(index, true, `key-${event.key}`); }
});
document.addEventListener('keyup', event => { const index = held.get(`key-${event.key}`); if (index !== undefined) { event.preventDefault(); button(index, false, `key-${event.key}`); } });
function touch(event, down) {
  if (!running || !DEVICES[deviceId()].touch) return;
  event.preventDefault(); const rect = $('screen').getBoundingClientRect();
  const x = Math.max(0, Math.min($('screen').width - 1, Math.round((event.clientX - rect.left) / rect.width * $('screen').width)));
  const y = Math.max(0, Math.min($('screen').height - 1, Math.round((event.clientY - rect.top) / rect.height * $('screen').height)));
  // Invert the clockwise panel-to-portrait rotation used for display pixels.
  send({ type: 'touch', x: y, y: $('screen').width - 1 - x, down });
}
$('screen').addEventListener('pointerdown', event => { $('screen').setPointerCapture(event.pointerId); touch(event, true); });
$('screen').addEventListener('pointermove', event => { if (event.buttons) touch(event, true); });
for (const kind of ['pointerup', 'pointercancel', 'lostpointercapture']) $('screen').addEventListener(kind, event => touch(event, false));
// GT911 home is its capacitive key register, represented by the SDK's negative
// coordinates in the virtual digitizer (see I2cDevices.cpp).
$('home-key').addEventListener('pointerdown', event => { event.preventDefault(); $('home-key').setPointerCapture(event.pointerId); send({ type: 'touch', x: -1, y: -1, down: true }); });
for (const kind of ['pointerup', 'pointercancel', 'lostpointercapture']) $('home-key').addEventListener(kind, () => send({ type: 'touch', x: -1, y: -1, down: false }));
$('home-key').addEventListener('click', event => { if (event.detail === 0) { send({ type: 'touch', x: -1, y: -1, down: true }); setTimeout(() => send({ type: 'touch', x: -1, y: -1, down: false }), 140); } });
$('capture').addEventListener('click', () => { if (!frame) { status('Wait for the first display refresh before capturing.'); return; } $('screen').toBlob(blob => { const link = document.createElement('a'); link.href = URL.createObjectURL(blob); link.download = `${$('app').value}-${deviceId()}.png`; link.click(); setTimeout(() => URL.revokeObjectURL(link.href), 1000); }); });
$('card-file').addEventListener('change', async event => { for (const file of event.target.files) { const bytes = await file.arrayBuffer(); cardFiles.set(file.name, bytes); worker?.postMessage({ type: 'file', name: file.name, bytes }); } });
renderDevice();
