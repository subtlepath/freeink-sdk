import test from 'node:test';
import assert from 'node:assert/strict';
import { DEVICES, portraitPixels, availableButton } from '../public/devices.mjs';
test('only fitted keys are exposed on each device', () => {
  assert.equal(DEVICES.X3.buttons.length,7);
  assert.equal(DEVICES.X4CLASSIC.buttons.length,7);
  assert.equal(DEVICES.X4PRO.buttons.length,3);
  assert.equal(DEVICES.X3.buttons.find(k=>k.index===6).edge,'top');
  assert.equal(DEVICES.X4CLASSIC.buttons.find(k=>k.index===6).edge,'right');
  for (const index of [0,1,2,3]) assert.equal(availableButton('X4PRO',index),false);
  assert.equal(DEVICES.X4PRO.touch,true);
});
test('landscape panel pixels rotate clockwise to match the physical portrait screen', () => {
  const result = portraitPixels(new Uint8Array([1,2,3,4,5,6]),3,2);
  assert.deepEqual([result.width,result.height],[2,3]);
  assert.deepEqual([...result.rgba].filter((_,i)=>i%4===0),[4,1,5,2,6,3]);
});
