function decodeUplink(input) {
  const b = input.bytes;
  if (b.length !== 6) return { errors: ["invalid length"] };

  let t = (b[0] << 8) | b[1];
  if (t & 0x8000) t -= 0x10000;
  const h = (b[2] << 8) | b[3];
  const v = (b[4] << 8) | b[5];

  return {
    data: {
      temperature_c: t / 100,
      humidity_rh: h / 100,
      battery_v: v / 1000
    }
  };
}
