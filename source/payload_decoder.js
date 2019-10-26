function Decoder(b, port) {

  var temp = (b[0] | b[1]<<8 | (b[1] & 0x80 ? 0xFF<<16 : 0)) / 100;
  var hum = (b[2] | b[3]<<8 | (b[3] & 0x80 ? 0xFF<<16 : 0)) / 100;
  var iaq = (b[4] | b[5]<<8 | (b[5] & 0x80 ? 0xFF<<16 : 0)) / 100;
  var press = (b[6] | b[7]<<8 | (b[7] & 0x80 ? 0xFF<<16 : 0))/10+7;
  var co2 = (b[8] | b[9]<<8 | (b[9] & 0x80 ? 0xFF<<16 : 0));


  if (co2 !== 0) {
    return {
    env: {
      temp: temp,
      hum: hum,
      iaq: iaq,
      press: press,
      co2: co2
    },
  };
  }
}
