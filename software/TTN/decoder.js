function Decoder(b, port) {
  var decoded = {};

  decoded.lat = (b[0] | b[1]<<8 | b[2]<<16 | (b[2] & 0x80 ? 0xFF<<24 : 0)) / 10000;
  decoded.lon = (b[3] | b[4]<<8 | b[5]<<16 | (b[5] & 0x80 ? 0xFF<<24 : 0)) / 10000;
  decoded.alt = (b[6] | b[7]<<8 | (b[7] & 0x80 ? 0xFFFF<<16 : 0));
  decoded.hdop = b[8] / 100;

  return decoded;
}