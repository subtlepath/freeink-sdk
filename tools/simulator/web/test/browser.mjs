import { chromium } from '@playwright/test';
import assert from 'node:assert/strict';
import fs from 'node:fs';

const base = process.env.FSIM_TEST_URL || 'http://127.0.0.1:8766';
const browser = await chromium.launch({ executablePath: process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE || (process.platform === 'darwin' ? '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome' : undefined), headless: true });
try {
  const page = await browser.newPage({ viewport: { width: 1280, height: 1000 } });
  const errors = [];
  page.on('pageerror', e => errors.push(e.message));
  page.on('console', m => { if (m.type() === 'error') console.log('console:', m.text()); });
  await page.goto(base);
  // A hosted lab loads lab.mjs slower than localhost; clicking first loses the Start handler.
  await page.waitForLoadState('networkidle');
  assert.equal(await page.evaluate(() => crossOriginIsolated), true);
  for (const device of (process.env.FSIM_TEST_DEVICES || 'X3').split(',')) {
    await page.locator(`input[name=device][value=${device}]`).check();
    if (process.env.FSIM_TEST_APP) await page.locator('#app').selectOption(process.env.FSIM_TEST_APP);
    await page.locator('#start').click();
    try {
      await page.waitForFunction(() => /refresh [1-9]/.test(document.getElementById('status').textContent), null, { timeout: 60000 });
    } catch (error) {
      console.log('Status:', await page.locator('#status').textContent());
      console.log('Firmware:', await page.locator('#console').textContent());
      console.log('Page errors:', errors);
      await page.screenshot({ path: '/tmp/freeink-web-failure.png', fullPage: true });
      throw error;
    }
    console.log(device, await page.locator('#status').textContent());
    await page.waitForFunction(app => document.getElementById('console').textContent.includes(app === 'lila' ? 'Entering activity: Library' : '[tinta] input live'), process.env.FSIM_TEST_APP || 'tinta', { timeout: 20000 });
    await page.waitForTimeout(2200);
    console.log((await page.locator('#console').textContent()).slice(-2800));
    await page.screenshot({ path: `/tmp/freeink-web-${device}.png`, fullPage: true });
    const pixels = await page.locator('#screen').evaluate(c => [...c.getContext('2d').getImageData(0,0,c.width,c.height).data].filter((v,i) => i%4!==3 && v<128).length);
    assert.ok(pixels > 500, 'Actual firmware must paint content, not a blank panel');
    // A page/navigation press must be noticed and produce a new panel refresh.
    const before = await page.locator('#status').textContent();
    const key = page.getByRole('button', { name: 'Next page', exact: true });
    await key.scrollIntoViewIfNeeded();
    const box = await key.boundingBox();
    await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
    await page.mouse.down();
    await page.waitForTimeout(160);
    await page.mouse.up();
    await page.waitForFunction(before => document.getElementById('status').textContent !== before, before, { timeout: 12000 });
    console.log('after input:', await page.locator('#status').textContent());
    console.log((await page.locator('#console').textContent()).slice(-900));
  }
  await page.setViewportSize({ width: 390, height: 844 });
  await page.screenshot({ path: '/tmp/freeink-web-mobile.png', fullPage: true });
  assert.equal(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth), true, 'No horizontal scroll on mobile');
  assert.deepEqual(errors, []);
} finally { await browser.close(); }
