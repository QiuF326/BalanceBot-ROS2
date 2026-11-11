/* Streaming resampler: area-average from the actual AudioContext sample rate.
 * Preserve fractional sample area between callbacks; emit exactly 320 samples.
 * This avoids assuming phones honour a requested 16 kHz capture rate.
 */
'use strict';
class PCM16Stream extends AudioWorkletProcessor {
  constructor() {
    super();
    this.ratio = sampleRate / 16000;
    this.remaining = this.ratio;
    this.sum = 0;
    this.buffer = new ArrayBuffer(640);
    this.view = new DataView(this.buffer);
    this.index = 0;
  }
  process(inputs, outputs) {
    for (const output of outputs) for (const channel of output) channel.fill(0);
    const samples = inputs[0] && inputs[0][0];
    if (!samples) return true;
    for (let i = 0; i < samples.length; ++i) {
      let amount = 1;
      while (amount > 1e-9) {
        const take = Math.min(amount, this.remaining);
        this.sum += samples[i] * take;
        this.remaining -= take;
        amount -= take;
        if (this.remaining < 1e-9) {
          const value = Math.max(-1, Math.min(1, this.sum / this.ratio));
          this.view.setInt16(this.index++ * 2, Math.round(value * (value < 0 ? 32768 : 32767)), true);
          this.sum = 0;
          this.remaining = this.ratio;
          if (this.index === 320) {
            this.port.postMessage({pcm:this.buffer, at:currentTime + i / sampleRate}, [this.buffer]);
            this.buffer = new ArrayBuffer(640);
            this.view = new DataView(this.buffer);
            this.index = 0;
          }
        }
      }
    }
    return true;
  }
}
registerProcessor('pcm16-stream', PCM16Stream);
