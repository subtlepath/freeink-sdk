import { chromium } from '@playwright/test';
import assert from 'node:assert/strict';
import fs from 'node:fs';

const base = process.env.FSIM_TEST_URL || 'http://127.0.0.1:8767/e-ink/';
const devices = (process.env.FSIM_TEST_DEVICES || process.argv[2] || 'X3,X4CLASSIC,X4PRO').split(',');
const browser = await chromium.launch({ executablePath: process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE || (process.platform === 'darwin' ? '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome' : undefined), headless: true });
try {
  const page = await browser.newPage({ viewport: { width: 1280, height: 1000 } });
  const errors = []; page.on('pageerror', e => errors.push(e.message));
  async function press(label) {
    const key = page.getByRole('button', { name: label, exact: true });
    await key.scrollIntoViewIfNeeded(); const box = await key.boundingBox();
    await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
    await page.mouse.down(); await page.waitForTimeout(160); await page.mouse.up();
    await page.waitForTimeout(2200);
  }
  async function tap(x, y) {
    const canvas = page.locator('#screen'); await canvas.scrollIntoViewIfNeeded();
    const box = await canvas.boundingBox(); const size = await canvas.evaluate(c => [c.width, c.height]);
    await page.mouse.move(box.x + x / size[0] * box.width, box.y + y / size[1] * box.height);
    await page.mouse.down(); await page.waitForTimeout(160); await page.mouse.up();
  }
  for (const device of devices) {
    await page.goto(base); await page.locator('#app').selectOption('lila');
    await page.locator(`input[value=${device}]`).check(); await page.locator('#start').click();
    await page.waitForFunction(() => document.getElementById('console').textContent.includes('reconciled:'), null, { timeout: 60000 });
    await page.waitForTimeout(2500);
    await page.locator('#screen').screenshot({ path: `/tmp/freeink-reader-library-${device}.png` });
    if (device === 'X4PRO') await tap(240, 185);
    else { await press('Next page'); await press('Confirm'); }
    await page.waitForFunction(() => /Entering activity: (EpubReader|Reader)/.test(document.getElementById('console').textContent), null, { timeout: 20000 });
    await page.waitForTimeout(10000);
    let previousImage;
    for (const phase of ['opened', 'next', 'previous']) {
      if (phase !== 'opened') {
        await press(device === 'X4PRO' ? (phase === 'next' ? 'Next page' : 'Previous page')
          : (phase === 'next' ? 'Next / right' : 'Previous / left'));
        await page.waitForTimeout(8000);
      }
      const logs = await page.locator('#console').textContent();
      fs.writeFileSync(`/tmp/freeink-reader-${device}.log`, logs);
      await page.locator('#screen').screenshot({ path: `/tmp/freeink-reader-${device}-${phase}.png` });
      const shades = await page.locator('#screen').evaluate(c => {
        const data = c.getContext('2d').getImageData(0, 0, c.width, c.height).data;
        let dark = 0, light = 0, gray = 0;
        for (let i = 0; i < data.length; i += 4) {
          if (data[i] < 32) dark++;
          else if (data[i] > 220) light++;
          else gray++;
        }
        return { dark, light, gray, total: c.width * c.height };
      });
      console.log(device, phase, shades);
      assert.ok(shades.light / shades.total > 0.65, 'Book paper must remain light after text smoothing');
      assert.ok(shades.dark > 1000, 'Book text must remain visible');
      // Some bundled fonts are 1bpp, so an empty AA mask is valid. The native
      // panel test exercises all grayscale codes with the real driver LUTs.
      const image = await page.locator('#screen').evaluate(c => {
        // A compact checksum keeps assertion failures readable.
        const pixels = c.getContext('2d').getImageData(0, 0, c.width, c.height).data;
        let hash = 2166136261;
        for (const value of pixels) hash = Math.imul(hash ^ value, 16777619);
        return hash >>> 0;
      });
      if (previousImage) assert.notEqual(image, previousImage, 'Hardware page key must change the displayed book page');
      previousImage = image;
    }
  }
  assert.deepEqual(errors, []);
} finally { await browser.close(); }
