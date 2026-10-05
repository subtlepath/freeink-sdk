// RFC 1321 MD5, used only for esptool's flash readback verification. Release
// authenticity/integrity before writing uses SHA-256, not this legacy digest.
const shifts = [7,12,17,22,5,9,14,20,4,11,16,23,6,10,15,21];
const constants = Int32Array.from({length:64}, (_, i) => Math.floor(Math.abs(Math.sin(i+1))*2**32));
const rotate = (x, n) => (x << n) | (x >>> (32-n));
export function md5(input) {
  const data = new Uint8Array(Math.ceil((input.length + 9) / 64) * 64);
  data.set(input); data[input.length] = 128;
  const view = new DataView(data.buffer);
  view.setUint32(data.length-8, (input.length*8) >>> 0, true);
  view.setUint32(data.length-4, Math.floor(input.length / 2**29), true);
  let a0 = 0x67452301 | 0, b0 = 0xefcdab89 | 0, c0 = 0x98badcfe | 0, d0 = 0x10325476 | 0;
  for (let offset=0; offset<data.length; offset+=64) {
    let a=a0, b=b0, c=c0, d=d0;
    for (let i=0; i<64; i++) {
      let f, g;
      if (i<16) { f=(b&c)|(~b&d); g=i; }
      else if (i<32) { f=(d&b)|(~d&c); g=(5*i+1)%16; }
      else if (i<48) { f=b^c^d; g=(3*i+5)%16; }
      else { f=c^(b|~d); g=(7*i)%16; }
      const step = (a+f+constants[i]+view.getInt32(offset+4*g,true))|0;
      a=d; d=c; c=b; b=(b+rotate(step,shifts[4*Math.floor(i/16)+i%4]))|0;
    }
    a0=(a0+a)|0; b0=(b0+b)|0; c0=(c0+c)|0; d0=(d0+d)|0;
  }
  const digest = new Uint8Array(16), output = new DataView(digest.buffer);
  [a0,b0,c0,d0].forEach((value,i)=>output.setInt32(4*i,value,true));
  return [...digest].map(x=>x.toString(16).padStart(2,'0')).join('');
}
