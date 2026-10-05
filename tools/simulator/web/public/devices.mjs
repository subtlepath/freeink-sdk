// Physical locations follow Xteink's September 2026 user-guide diagrams.
// Indices are the SDK ABI: back, confirm, left, right, up, down, power.
// Case dimensions are manufacturer specs. Bezel positions are normalized
// from diagrams, not claimed to be mechanical CAD measurements.
export const DEVICES = {
  X3: { name: 'X3', chip: 'esp32c3', width: 63.7, height: 97.6, screen: [528, 792], touch: false, reset: 19, buttons: [
    { index: 6, label: 'Power', edge: 'top', position: 70 },
    { index: 4, label: 'Previous page', edge: 'left', position: 26 },
    { index: 5, label: 'Next page', edge: 'right', position: 26 },
    { index: 0, label: 'Back', edge: 'bottom', position: 23 },
    { index: 1, label: 'Confirm', edge: 'bottom', position: 41 },
    { index: 2, label: 'Previous / left', edge: 'bottom', position: 77 },
    { index: 3, label: 'Next / right', edge: 'bottom', position: 59 },
  ]},
  X4CLASSIC: { name: 'X4 Classic', chip: 'esp32s3', width: 69, height: 114, screen: [480, 800], touch: false, reset: 80, buttons: [
    { index: 6, label: 'Power', edge: 'right', position: 12 },
    { index: 4, label: 'Previous page', edge: 'left', position: 27 },
    { index: 5, label: 'Next page', edge: 'right', position: 27 },
    { index: 0, label: 'Back', edge: 'bottom', position: 23 },
    { index: 1, label: 'Confirm', edge: 'bottom', position: 41 },
    { index: 2, label: 'Previous / left', edge: 'bottom', position: 77 },
    { index: 3, label: 'Next / right', edge: 'bottom', position: 59 },
  ]},
  X4PRO: { name: 'X4 Pro', chip: 'esp32s3', width: 69, height: 111, screen: [480, 800], touch: true, reset: 80, buttons: [
    { index: 6, label: 'Power', edge: 'right', position: 12 },
    { index: 4, label: 'Previous page', edge: 'left', position: 29 },
    { index: 5, label: 'Next page', edge: 'right', position: 29 },
  ]},
};
export const KEY_BUTTONS = { Escape: 0, Enter: 1, ArrowLeft: 2, ArrowRight: 3, ArrowUp: 4, ArrowDown: 5, p: 6, P: 6 };
export function availableButton(device, index) { return DEVICES[device].buttons.some(button => button.index === index); }
// Panel RAM is landscape; firmware's portrait coordinates rotate clockwise.
export function portraitPixels(pixels, width, height) {
  const rgba = new Uint8ClampedArray(width * height * 4);
  for (let y = 0; y < height; y++) for (let x = 0; x < width; x++) {
    const target = (x * height + (height - 1 - y)) * 4;
    const shade = pixels[y * width + x];
    rgba[target] = rgba[target + 1] = rgba[target + 2] = shade;
    rgba[target + 3] = 255;
  }
  return { rgba, width: height, height: width };
}
