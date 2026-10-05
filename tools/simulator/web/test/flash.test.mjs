import test from 'node:test';
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { validatePlan, verifiedFiles, installVerified } from '../public/flash-plan.mjs';
import { md5 } from '../public/md5.mjs';
const bytes = new Uint8Array([1,2,3,4]);
const digest = createHash('sha256').update(bytes).digest('hex');
const plan = { app:'tinta',device:'X4CLASSIC',chip:'esp32s3',preserve:[{start:0x9000,end:0xe000}],files:[{address:0x10000,size:4,sha256:digest,url:'test.bin'}] };
const fetcher = async () => new Response(bytes);
test('only the exact board, software and detected chip can use a plan', () => {
  assert.equal(validatePlan(plan,'tinta','X4CLASSIC','esp32s3').length,1);
  for (const args of [['lila','X4CLASSIC','esp32s3'],['tinta','X4PRO','esp32s3'],['tinta','X4CLASSIC','esp32c3']]) assert.throws(()=>validatePlan(plan,...args));
});
test('erase-sector boundaries preserve factory NVS, reject overlaps and out-of-range writes', () => {
  assert.throws(()=>validatePlan({...plan,preserve:[],files:[{...plan.files[0],address:0x9000}]},'tinta','X4CLASSIC','esp32s3'));
  for (const address of [0x8fff,0x9000,0xdfff,-1,0x1000000]) assert.throws(()=>validatePlan({...plan,files:[{...plan.files[0],address}]},'tinta','X4CLASSIC','esp32s3'));
  assert.throws(()=>validatePlan({...plan,files:[plan.files[0],{...plan.files[0],address:0x10004}]},'tinta','X4CLASSIC','esp32s3'));
});
test('all files must pass SHA-256 and length before writeFlash is called', async () => {
  assert.equal((await verifiedFiles(plan,'tinta','X4CLASSIC','esp32s3',fetcher))[0].data.length,4);
  let writes = 0;
  const loader = { writeFlash:async()=>writes++,after:async()=>{} };
  await assert.rejects(installVerified(loader,plan,'tinta','X4CLASSIC','esp32s3',()=>{},async()=>new Response(new Uint8Array([9,8,7,6]))),/checksum/);
  assert.equal(writes,0);
  await assert.rejects(installVerified(loader,plan,'tinta','X4CLASSIC','esp32s3',()=>{},async()=>new Response(new Uint8Array([9]))),/length/);
  assert.equal(writes,0);
});
test('installation keeps flash configuration, avoids mass erase and verifies device MD5 before reset', async () => {
  const calls=[];
  await installVerified({writeFlash:async options=>{calls.push('write');assert.equal(options.eraseAll,false);assert.equal(options.flashMode,'keep');assert.equal(options.calculateMD5Hash(bytes),md5(bytes));},after:async mode=>{calls.push(mode);}},plan,'tinta','X4CLASSIC','esp32s3',()=>{},fetcher);
  assert.deepEqual(calls,['write','hard_reset']);
});
test('MD5 agrees with the platform implementation across block/padding boundaries', () => {
  for (const size of [0,1,3,55,56,63,64,65,1000,65537]) {
    const data = Uint8Array.from({length:size},(_,i)=>i%251);
    assert.equal(md5(data),createHash('md5').update(data).digest('hex'));
  }
});
