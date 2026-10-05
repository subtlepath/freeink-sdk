import { DEVICES } from './devices.mjs';
import { installVerified } from './flash-plan.mjs';

const $ = id => document.getElementById(id);
let transport, loader, connectedChip, releases = [], selection, writing = false;
const log = text => { $('flash-log').textContent = ($('flash-log').textContent + text + '\n').slice(-20000); };
const status = text => { $('flash-status').textContent = text; };
const supported = isSecureContext && 'serial' in navigator;
$('serial-support').textContent = supported ? 'Web Serial is available. No desktop flasher needed.' : 'For USB installation, use desktop Chrome or Edge over HTTPS. You can still try the simulator here.';
$('connect').disabled = !supported;

function update() {
  selection = releases.find(r => r.app === $('flash-app').value && r.device === $('flash-device').value);
  $('release-info').textContent = selection ? `${selection.title} · ${selection.version}${selection.draft ? ' · development build' : ''} · ${DEVICES[selection.device].name}. Factory NVS is preserved.` : 'No packaged release for this combination. Build and package that exact target first.';
  if (selection?.source_url) {
    const link = document.createElement('a'); link.href = `./releases/${selection.source_url}`; link.textContent = 'Source & build recipes';
    $('release-info').append(document.createElement('br'), link, ' · ');
    const notices = document.createElement('a'); notices.href = `./releases/licenses/${selection.app === 'tinta' ? 'NOTICE' : 'READER-NOTICE.txt'}`; notices.textContent = 'Licenses';
    $('release-info').append(notices);
  }
  $('write').disabled = writing || !loader || !selection || !$('flash-consent').checked || connectedChip !== selection.chip;
  $('connect').disabled = !supported || writing || Boolean(loader);
  $('disconnect').disabled = writing || !transport;
  for (const id of ['flash-app', 'flash-device', 'flash-consent']) $(id).disabled = writing;
}
for (const id of ['flash-app', 'flash-device']) $(id).addEventListener('change', () => { $('flash-consent').checked = false; update(); });
$('flash-consent').addEventListener('change', update);

async function disconnect() {
  try { await transport?.disconnect(); } catch (error) { log(error.message); }
  finally { transport = loader = connectedChip = undefined; update(); }
}
$('disconnect').addEventListener('click', async () => { await disconnect(); status('Disconnected.'); });
$('connect').addEventListener('click', async () => {
  $('connect').disabled = true;
  try {
    const port = await navigator.serial.requestPort(); // Direct user gesture.
    const { ESPLoader, Transport } = await import('./vendor/esptool.js');
    transport = new Transport(port, true);
    loader = new ESPLoader({ transport, baudrate: 460800, terminal: { clean() {}, write: log, writeLine: log } });
    status('Connecting and identifying the chip…');
    const detected = await loader.main();
    connectedChip = /^ESP32-C3\b/i.test(detected) ? 'esp32c3' : /^ESP32-S3\b/i.test(detected) ? 'esp32s3' : 'unsupported';
    if (connectedChip !== DEVICES[$('flash-device').value].chip) throw new Error(`Detected ${detected}; that chip does not match ${DEVICES[$('flash-device').value].name}.`);
    status(`Connected: ${detected}. Check the exact model and backup before installing.`);
  } catch (error) { await disconnect(); status(error.name === 'NotFoundError' ? 'Device selection cancelled.' : error.message); log(error.message); }
  finally { update(); }
});
$('write').addEventListener('click', async () => {
  if (!selection || !loader || !$('flash-consent').checked || writing) return;
  const plan = selection, app = $('flash-app').value, device = $('flash-device').value;
  writing = true; update();
  try {
    status('Downloading and checking every firmware region…');
    const total = plan.files.reduce((sum, f) => sum + f.size, 0);
    const offsets = plan.files.map((_, i) => plan.files.slice(0, i).reduce((sum, f) => sum + f.size, 0));
    await installVerified(loader, plan, app, device, connectedChip, (index, written, compressedTotal) => {
      const percent = Math.min(100, Math.round((offsets[index] + plan.files[index].size * written / Math.max(1, compressedTotal)) / total * 100));
      $('flash-progress').value = percent;
      status(`Installing ${plan.title} on ${DEVICES[device].name} · ${percent}%`);
    });
    $('flash-progress').value = 100;
    status('Installation finished. The device is restarting.');
  } catch (error) { status(`Installation stopped: ${error.message}`); log(error.message); }
  finally { writing = false; await disconnect(); update(); }
});
if (supported) navigator.serial.addEventListener('disconnect', async () => {
  if (writing) status('USB disconnected during installation. Reconnect in download mode and install again.');
  else { await disconnect(); status('USB device disconnected.'); }
});
try {
  const response = await fetch('./releases/index.json');
  if (!response.ok) throw new Error('No release manifest is installed.');
  releases = await response.json();
} catch (error) { log(error.message); }
update();
