export const FLASH_BYTES = 16 * 1024 * 1024;
export function validatePlan(plan, app, device, chip) {
  if (!plan || plan.app !== app || plan.device !== device) throw new Error('Release does not match the selected software and device.');
  if (plan.chip !== chip) throw new Error(`Wrong chip: firmware needs ${plan.chip}; connected device is ${chip}.`);
  if (!Array.isArray(plan.files) || !plan.files.length) throw new Error('Release has no flash regions.');
  const files = [...plan.files].sort((a, b) => a.address - b.address);
  let end = 0;
  for (const file of files) {
    if (!Number.isInteger(file.address) || !Number.isInteger(file.size) || file.address < 0 || file.size <= 0 || file.address + file.size > FLASH_BYTES) throw new Error('Invalid flash address or length.');
    // Flash erase granularity is 4 KiB. Also forbid shared erase sectors.
    const startSector = Math.floor(file.address / 4096) * 4096;
    const endSector = Math.ceil((file.address + file.size) / 4096) * 4096;
    if (startSector < end) throw new Error('Flash regions overlap or share an erase sector.');
    for (const region of [{ start: 0x9000, end: 0xe000 }, ...(plan.preserve ?? [])]) {
      if (startSector < region.end && endSector > region.start) throw new Error('Release would erase protected factory data.');
    }
    if (!/^[a-f0-9]{64}$/.test(file.sha256)) throw new Error('Release is missing a SHA-256 checksum.');
    if (typeof file.url !== 'string' || file.url.startsWith('/') || file.url.includes('..') || /^[a-z]+:/i.test(file.url)) throw new Error('Release must use local relative files.');
    end = endSector;
  }
  return files;
}
export async function verifiedFiles(plan, app, device, chip, fetcher = fetch) {
  const files = validatePlan(plan, app, device, chip);
  return Promise.all(files.map(async file => {
    const response = await fetcher(new URL(file.url, new URL('./releases/', import.meta.url)));
    if (!response.ok) throw new Error(`Firmware download failed (${response.status}).`);
    const bytes = new Uint8Array(await response.arrayBuffer());
    if (bytes.length !== file.size) throw new Error('Firmware length check failed.');
    const digest = new Uint8Array(await crypto.subtle.digest('SHA-256', bytes));
    const hash = [...digest].map(byte => byte.toString(16).padStart(2, '0')).join('');
    if (hash !== file.sha256) throw new Error('Firmware checksum failed. Nothing was written.');
    return { data: bytes, address: file.address };
  }));
}

export async function installVerified(loader, plan, app, device, chip, progress, fetcher = fetch) {
  // All downloads and checks finish before the first erase/write command.
  const fileArray = await verifiedFiles(plan, app, device, chip, fetcher);
  await loader.writeFlash({ fileArray, flashMode: 'keep', flashFreq: 'keep', flashSize: 'keep', eraseAll: false, compress: true, reportProgress: progress, calculateMD5Hash: md5 });
  await loader.after('hard_reset');
}
import { md5 } from './md5.mjs';
