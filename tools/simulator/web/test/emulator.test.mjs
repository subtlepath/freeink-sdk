import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { execFileSync } from 'node:child_process';
const runtime = new URL('../public/runtime/emulator.mjs',import.meta.url);
const simulator = fileURLToPath(new URL('../../',import.meta.url));
test('CPU WASM executes actual C3/S3 fixture instructions and rejects mismatched devices', {skip:!fs.existsSync(runtime)}, async () => {
  const { default:create } = await import(runtime.href);
  const directory = fs.mkdtempSync(path.join(os.tmpdir(),'freeink-web-cpu-'));
  try {
    for (const [chip,device] of [['esp32c3','X3'],['esp32s3','X4CLASSIC'],['esp32s3','X4PRO']]) {
      const image = path.join(directory,`${device}.bin`);
      execFileSync('python3',[path.join(simulator,'test/make-test-image.py'),'--chip',chip,image]);
      const m = await create(); m.FS.mkdir('/sd'); m.FS.mkdir('/rom');
      for (const soc of ['esp32c3','esp32s3']) m.FS.writeFile(`/rom/${soc}.romsyms`,fs.readFileSync(path.join(simulator,`emu/rom/${soc}.romsyms`)));
      m.FS.writeFile('/test.bin',fs.readFileSync(image));
      assert.equal(m.ccall('web_load','number',['string','string'],['/test.bin',chip==='esp32c3'?'X4PRO':'X3']),0);
      assert.equal(m.ccall('web_load','number',['string','string'],['/test.bin',device]),1);
      m._web_run(10000);
      assert.match(m.ccall('web_logs','string',[],[]),/FreeInk emulator fixture/);
    }
  } finally { fs.rmSync(directory,{recursive:true,force:true}); }
});
