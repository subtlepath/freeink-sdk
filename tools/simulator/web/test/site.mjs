import { chromium } from '@playwright/test';
import assert from 'node:assert/strict';
const root = process.env.FSIM_SITE_URL || 'http://127.0.0.1:8767';
const browser = await chromium.launch({ executablePath: process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE || (process.platform === 'darwin' ? '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome' : undefined), headless: true });
try {
  const page = await browser.newPage({ viewport: { width: 1280, height: 1000 } });
  const errors=[]; page.on('pageerror', e => errors.push(e.message));
  const response = await page.goto(root);
  assert.ok(!response.headers()['content-security-policy']?.includes('upgrade-insecure-requests'), 'HTTP preview must not upgrade assets to HTTPS');
  assert.equal(response.headers()['strict-transport-security'], undefined);
  for (const path of ['/assets/site.css', '/assets/eink.css', '/e-ink/lab.css']) {
    const css = await page.request.get(`${root}${path}`);
    assert.equal(css.status(), 200, `${path} must load`);
    assert.match(css.headers()['content-type'], /text\/css/);
  }
  assert.equal(await page.evaluate(() => [...document.styleSheets].some(sheet => sheet.href?.endsWith('/assets/site.css') && sheet.cssRules.length > 0)), true, 'Homepage stylesheet must be applied');
  assert.match(await page.locator('#eink-heading').textContent(), /small screen/);
  await page.locator('#e-ink').screenshot({ path: '/tmp/freeink-sp2-feature.png' });
  await page.setViewportSize({ width: 390, height: 844 });
  await page.screenshot({ path:'/tmp/freeink-sp2-mobile.png',fullPage:true });
  assert.equal(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth), true, 'Homepage must fit mobile');
  await page.emulateMedia({ colorScheme:'dark' });
  await page.locator('#e-ink').screenshot({ path:'/tmp/freeink-sp2-dark.png' });
  await page.setViewportSize({ width:1280,height:1000 });
  await page.getByRole('link', { name: 'Try the live e-ink simulator', exact: true }).click();
  await page.waitForURL(`${root}/e-ink/`);
  assert.equal(await page.evaluate(() => crossOriginIsolated), true);
  const plans = await (await page.request.get(`${root}/e-ink/releases/index.json`)).json();
  assert.equal(plans.length, 6);
  for (const plan of plans) {
    await page.locator('#flash-device').selectOption(plan.device); await page.locator('#flash-app').selectOption(plan.app);
    assert.match(await page.locator('#release-info').textContent(), new RegExp(plan.version.replaceAll('.', '\\.')));
    assert.equal(await page.locator('#write').isDisabled(), true);
  }
  await page.locator('#app').selectOption('lila'); await page.locator('#start').click();
  await page.waitForFunction(() => document.getElementById('console').textContent.includes('reconciled:'), null, { timeout:60000 });
  await page.getByText('Virtual SD card',{exact:true}).click();
  await page.locator('#card-file').setInputFiles({name:'Visitor.txt',mimeType:'text/plain',buffer:Buffer.from('A visitor brought this text to the virtual card.\n'.repeat(80))});
  await page.waitForFunction(() => document.getElementById('console').textContent.includes('Added Visitor.txt'), null, { timeout:5000 });
  await page.locator('#restart').click();
  try { await page.waitForFunction(() => /built: 2 books/.test(document.getElementById('console').textContent), null, {timeout:60000}); }
  catch (error) { console.log(await page.locator('#status').textContent()); console.log(await page.locator('#console').textContent()); console.log(errors); throw error; }
  assert.deepEqual(errors,[]);
  console.log('Homepage, mobile/dark layouts, six install selections and virtual-card restart passed.');
} finally { await browser.close(); }
