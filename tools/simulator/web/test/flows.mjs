import { chromium } from '@playwright/test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
const base = process.env.FSIM_TEST_URL || 'http://127.0.0.1:8766';
const browser = await chromium.launch({ executablePath: process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE || (process.platform === 'darwin' ? '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome' : undefined), headless: true });
const page = await browser.newPage({ viewport: { width: 1280, height: 1000 } });
const errors = []; page.on('pageerror', e => errors.push(e.message));
async function boot(app, device) {
  await page.goto(base); await page.locator('#app').selectOption(app);
  await page.locator(`input[value=${device}]`).check(); await page.locator('#start').click();
  await page.waitForFunction(app => document.getElementById('console').textContent.includes(app === 'lila' ? 'Entering activity: Library' : '[tinta] input live'), app, { timeout: 60000 });
  await page.waitForTimeout(2500);
}
async function tap(x, y) {
  await page.locator('#screen').scrollIntoViewIfNeeded(); const box = await page.locator('#screen').boundingBox();
  const size = await page.locator('#screen').evaluate(c => [c.width,c.height]);
  await page.mouse.move(box.x + x / size[0] * box.width, box.y + y / size[1] * box.height);
  await page.mouse.down(); await page.waitForTimeout(150); await page.mouse.up(); await page.waitForTimeout(2300);
}
async function press(label) {
  const key = page.getByRole('button', { name: label, exact: true }); await key.scrollIntoViewIfNeeded(); const box = await key.boundingBox();
  await page.mouse.move(box.x + box.width/2, box.y + box.height/2); await page.mouse.down(); await page.waitForTimeout(150); await page.mouse.up(); await page.waitForTimeout(2300);
}
async function capture(name) {
  const encoded = await page.locator('#screen').evaluate(c => c.toDataURL().split(',')[1]);
  fs.writeFileSync(new URL(`../public/content/${name}-screen.png`, import.meta.url), Buffer.from(encoded,'base64'));
  await page.screenshot({ path: `/tmp/freeink-flow-${name}.png`, fullPage: true });
}
try {
  await boot('tinta', 'X4PRO'); await tap(240,349);
  console.log('Tinta after tap:', (await page.locator('#console').textContent()).slice(-2200));
  assert.match(await page.locator('#console').textContent(), /set uiLanguage english/, 'Touch must choose the exact English target');
  await press('Capacitive Home');
  console.log('Tinta after Home:', (await page.locator('#console').textContent()).slice(-1000));
  assert.match(await page.locator('#console').textContent(), /key home/);
  await tap(180,770);
  console.log('Tinta after Next:', (await page.locator('#console').textContent()).slice(-1000));
  const targets = [...(await page.locator('#console').textContent()).matchAll(/target [\w-]+\/start (\d+) (\d+)/g)];
  if (targets.length) { const target = targets.at(-1); await tap(Number(target[1]), Number(target[2])); }
  await capture('tinta');
  await boot('lila','X3'); await press('Next page'); await press('Confirm');
  console.log('Reader after confirm:', (await page.locator('#console').textContent()).slice(-2000));
  await page.waitForFunction(() => /Entering activity: (EpubReader|Reader)/.test(document.getElementById('console').textContent), null, { timeout: 20000 });
  await page.waitForTimeout(8000);
  console.log('Reader rendered:', (await page.locator('#console').textContent()).slice(-2000));
  await capture('lila');
  assert.deepEqual(errors, []);
} finally { await browser.close(); }
