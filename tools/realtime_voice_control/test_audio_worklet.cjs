// Run with Node.js. Exercise the actual worklet at common phone sample rates.
const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const path = require('node:path');
const source = fs.readFileSync(path.join(__dirname, 'pcm-worklet.js'), 'utf8');
for (const rate of [16000, 44100, 48000]) {
  const frames = [];
  let Processor;
  const context = vm.createContext({
    sampleRate:rate, currentTime:0, ArrayBuffer, DataView, Math,
    AudioWorkletProcessor:class {constructor(){this.port={postMessage:x=>frames.push(x)};}},
    registerProcessor:(name, processor)=>{Processor=processor;},
  });
  new vm.Script(source).runInContext(context);
  const processor = new Processor();
  const seconds = 5;
  for (let offset = 0; offset < rate * seconds; offset += 128) {
    const n = Math.min(128, rate * seconds - offset);
    const input = new Float32Array(n).fill(0.5);
    const output = new Float32Array(n).fill(1);
    context.currentTime = offset / rate;
    assert.equal(processor.process([[input]], [[output]]), true);
    assert.ok(output.every(x => x === 0), 'microphone must not play through speakers');
  }
  assert.equal(frames.length, 50 * seconds, `${rate}: no long-term sample drift`);
  for (const frame of frames) {
    assert.equal(frame.pcm.byteLength, 640);
    const view = new DataView(frame.pcm);
    for(let offset=0; offset<640; offset+=2) assert.ok(Math.abs(view.getInt16(offset,true)-16384)<=1);
  }
  console.log(`PASS ${rate} Hz -> 16000 Hz: ${frames.length} PCM20ms frames, no drift, silent output`);
}
const html = fs.readFileSync(path.join(__dirname, 'index.html'),'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
new vm.Script(script);
console.log('PASS phone page JavaScript syntax');
