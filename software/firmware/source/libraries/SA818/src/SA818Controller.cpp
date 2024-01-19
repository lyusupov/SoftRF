#include <Arduino.h>
#include <SA818Controller.h>

SA818Controller::SA818Controller(SA818* sa818) : sa(sa818) {}

SA818Controller::~SA818Controller() {
    sa = 0;
}

void SA818Controller::setModel(Model type) {
    type_ = type;
}

byte SA818Controller::getModel() {
    return type_;
}

void SA818Controller::setBand(Band band) {
    band_ = band;
}

byte SA818Controller::getBand() {
    return band_;
}

void SA818Controller::setPins(uint8_t ptt, uint8_t pd, uint8_t hl) {
    ptt_ = ptt;
    pd_  = pd;
    hl_  = hl;

    pinMode(pd_, OUTPUT);
    pinMode(hl_, OUTPUT);
    pinMode(ptt_, OUTPUT);
}

String SA818Controller::response() {
    return sa->response()->raw;
}
String SA818Controller::result() {
    return sa->response()->res;
}

void SA818Controller::setBW(byte bw) {
    bw_ = bw;
}

void SA818Controller::setTXF(float tx) {
    tx_f_ = tx;
}

float SA818Controller::getTXF() {
    return tx_f_;
}

void SA818Controller::setRXF(float rx) {
    rx_f_ = rx;
}

void SA818Controller::setTXSub(int tx) {
    tx_sub_ = tx;
}

void SA818Controller::setSQ(byte sq) {
    sq_ = sq;
}

void SA818Controller::setRXSub(int rx) {
    rx_sub_ = rx;
}

bool SA818Controller::update() {
    return setGroup(bw_, tx_f_, rx_f_, tx_sub_, sq_, rx_sub_);
}

String SA818Controller::ctcss(int sub) {
    String ssub = String(abs(sub));
    if (sub < 0)
        ssub += "N";
    if (sub >= 23)
        ssub += "I";

    String res = ssub;
    for (int i = ssub.length(); i < 4; i++) {
        res = "0" + res;
    }
    return String(res);
}

bool SA818Controller::connect() {
    return sa->send("AT+DMOCONNECT");
}

bool SA818Controller::setGroup(byte bw, float tx_f, float rx_f, int tx_sub, byte sq, int rx_sub) {

    setBW(bw);
    setTXF(tx_f);
    setRXF(rx_f);
    setTXSub(tx_sub);
    setSQ(sq);
    setRXSub(rx_sub);

    String b = String(bw);
    String tf = String(tx_f, 4);
    String rf = String(rx_f, 4);
    String tsub = ctcss(tx_sub);
    String s = String(sq);
    String rsub = ctcss(rx_sub);
    char* params[] = {
        (char *) b.c_str(),
        (char *) tf.c_str(),
        (char *) rf.c_str(),
        (char *) tsub.c_str(),
        (char *) s.c_str(),
        (char *) rsub.c_str()
    };
    return sa->send("AT+DMOSETGROUP", 6, params);
}

bool SA818Controller::scan(float freq) {
    String cmd = String("S+") + String(freq, 4);
    return sa->send((char *) cmd.c_str());
}

bool SA818Controller::setVolume(byte vol) {
    String v = String(vol);
    char* params[] = { (char *) v.c_str() };
    return sa->send("AT+DMOSETVOLUME", 1, params);
}

bool SA818Controller::setFilter(byte emph, byte high, byte low) {
    String e = String(emph);
    String h = String(high);
    String l = String(low);
    char* params[] = {(char *) e.c_str(), (char *) h.c_str(), (char *) l.c_str()};
    return sa->send("AT+SETFILTER", 3, params);
}

bool SA818Controller::openTail() {
    return sa->send("AT+SETTAIL=1");
}

bool SA818Controller::closeTail() {
    return sa->send("AT+SETTAIL=0");
}

String SA818Controller::rssi() {
    String cmd = "RSSI?";
    if (type_ == Model::SA_868)
        cmd = "AT+RSSI?";

    if (sa->send((char *) cmd.c_str()))
        return sa->response()->res;
    return sa->response()->raw;
}


String SA818Controller::version() {
    if (sa->send("AT+VERSION"))
        return sa->response()->res;
    return sa->response()->raw;
}

float SA818Controller::loopScan(float mhz, bool dir, float stop, unsigned long timeout) {
    // | GHz |     |     | MHz ||     |     | KHz ||     |     | Hz |
    // |     |     |     |     ||     |     |   1 ||   0 |   0 |  0 |
    // |     |     |     |   1 ||   0 |   0 |   0 ||   0 |   0 |  0 |
    // |   1 |   0 |   0 |   0 ||   0 |   0 |   0 ||   0 |   0 |  0 |
    // |     |   1 |   4 |   5 ||   7 |   5 |   0 ||   0 |   0 |  0 |
    // |     |     |     |     ||     |   1 |   2 ||   5 |   0 |  0 |

    if (stop == 0)
        stop = mhz;

    float khz = 12.5; // 12.5K
    if (bw_ == 1)
        khz = 25; // 25K

    // Out of frequencies bounds
    float top = 174000; // VHF KHz top limit
    float bottom = 134000;
    if (band_ == Band::UHF) {
        top = 480000; // UHF
        bottom = 400000;
    }

    unsigned long timer = millis();
    float f = mhz;
    bool next = true;
    while (next) {
        f *= 1000; // MHz to KHz

        if (dir) {
            f = f + khz;

            if (f > top) f = bottom; // out of top bound
        }
        else {
            f = f - khz;

            if (f < bottom) f = top; // out of bottom bound
        }
        f /= 1000;

        next = !scan(f);

        if(timeout != 0 && millis() - timer >= timeout) {
            next = false;
        }
        
        if (f == stop) {
            next = false;
        }
    }
    return f;
}

float SA818Controller::next(float mhz, float stop, unsigned long timeout) {
    if (stop == 0)
        stop = mhz;
    return loopScan(mhz, true, stop, timeout);
}

float SA818Controller::previous(float mhz, float stop, unsigned long timeout) {
    if (stop == 0)
        stop = mhz;
    return loopScan(mhz, false, stop, timeout);
}

int SA818Controller::scanlist(float list[], int size, float ret[]) {
    int r = 0;
    for (int i = 0; i < size; i++) {
        if (scan(list[i]))
            ret[r++] = list[i];
    }

    return r;
}

void SA818Controller::sleep()
{
    digitalWrite(pd_, LOW);
}

void SA818Controller::wake()
{
    digitalWrite(pd_, HIGH);
}

void SA818Controller::highPower()
{
    digitalWrite(hl_, HIGH);
}

void SA818Controller::lowPower()
{
    digitalWrite(hl_, LOW);
}

void SA818Controller::receive()
{
    if (!_transmitStatus) return ;
    _transmitStatus = false;
    digitalWrite(ptt_, HIGH);
}

void SA818Controller::transmit()
{
    if (_transmitStatus) return ;
    _transmitStatus = true;
    digitalWrite(ptt_, LOW);
}

bool SA818Controller::getTxStatus()
{
    return _transmitStatus;
}

/*
 * TODO
 *
 * Source: https://github.com/nakhonthai/ESP32APRS_T-TWR/pull/17
 */

bool SA868_WriteAT1846Sreg(HardwareSerial * SerialRF, uint8_t reg, uint16_t value)
{
    char str[200];
    String result;
    sprintf(str, "AT+POKE=%d,%d\r\n", reg, value);
    if (!SA868_WaitResponse(SerialRF, str, &result)) {
        ESP_LOGE("SA8x8", "Error: reg: %02X <- val: %02X", reg, value);
        return false;
    }

    return true;
}

uint16_t SA868_ReadAT1846Sreg(HardwareSerial * SerialRF, uint8_t reg)
{
    String data;
    uint16_t value = 0;
    char str[200];

    sprintf(str, "AT+PEEK=%d\r\n", reg);
    if (!SA868_WaitResponse(SerialRF, str, &data))
    {
        ESP_LOGD("SA8x8", "Error: reg: %02X -> \n", reg);
        return 0;
    }
    sscanf(data.c_str(), "%hd\r", &value);

    return value;
}

void SA868_maskSetRegister(HardwareSerial * SerialRF, const uint8_t reg, const uint16_t mask, const uint16_t value)
{
    uint16_t regVal = SA868_ReadAT1846Sreg(SerialRF, reg);
    regVal = (regVal & ~mask) | (value & mask);
    SA868_WriteAT1846Sreg(SerialRF, reg, regVal);
}

uint16_t SA868_maskSetValue(const uint16_t initValue, const uint16_t mask, const uint16_t value)
{
    return (initValue & ~mask) | (value & mask);
}

void SA868_reloadConfig(HardwareSerial * SerialRF)
{
    uint16_t funcMode = SA868_ReadAT1846Sreg(SerialRF, 0x30) & 0x0060;  // Get current op. status
    SA868_maskSetRegister(SerialRF, 0x30, 0x0060, 0x0000);              // RX and TX off
    SA868_maskSetRegister(SerialRF, 0x30, 0x0060, funcMode);            // Restore op. status
}

void OpenEdition::setAudio(bool value)
{
    char str[200];
    String result;
    sprintf(str, "AT+AUDIO=%d\r\n", value);
    if (!SA868_WaitResponse(_SerialRF, str, &result))
    {
        ESP_LOGD("SA8x8", "SetAudio Error");
    }
}

void SA868_setFuncMode(HardwareSerial * SerialRF, const OpenEdition_Mode mode)
{
    /*
     * Functional mode is controlled by bits 5 (RX on) and 6 (TX on) in
     * register 0x30. With a cast and shift we can set it easily.
     */

    uint16_t value = static_cast< uint16_t >(mode) << 5;
    SA868_maskSetRegister(SerialRF, 0x30, 0x0060, value);
}

OpenEdition::OpenEdition(HardwareSerial * SerialRF, uint8_t RX_PIN, uint8_t TX_PIN)
: _SerialRF {SerialRF}
, _RX_PIN {RX_PIN}
, _TX_PIN {TX_PIN}
{
  _config.freq_rx = 144800000;
  _config.freq_tx = 144800000;
  _config.tone_rx = 0;
  _config.tone_tx = 0;
  _config.band = 0;
  _config.sql_level = 80;
  _config.rf_power = true;
  _config.volume = 20;
  _config.mic = 8;
  _config.mode = OpenEdition_Mode::OFF;
}

void OpenEdition::setTxFrequency(uint32_t freq)
{
    // todo check for range
    _config.freq_tx = freq;
    if(_config.mode == OpenEdition_Mode::TX)
    {
        setFrequency(freq);
    }
}

void OpenEdition::setRxFrequency(uint32_t freq)
{
    // todo check for range
    _config.freq_rx = freq;
    if (_config.mode == OpenEdition_Mode::RX)
    {
        setFrequency(freq);
    }
}

void OpenEdition::setTxTone(uint32_t tone)
{
    _config.tone_tx = tone;
}

void OpenEdition::setRxTone(uint32_t tone)
{
    _config.tone_rx = tone;
}

void OpenEdition::init() {
    _SerialRF->begin(9600, SERIAL_8N1, _RX_PIN, _TX_PIN);

    SA868_WriteAT1846Sreg(_SerialRF, 0x30, 0x0001);   // Soft reset
    delay(50);
    SA868_WriteAT1846Sreg(_SerialRF, 0x30, 0x0004);   // Chip enable
    SA868_WriteAT1846Sreg(_SerialRF, 0x04, 0x0FD0);   // 26MHz crystal frequency
    SA868_WriteAT1846Sreg(_SerialRF, 0x1F, 0x1000);   // Gpio6 squelch output
    SA868_WriteAT1846Sreg(_SerialRF, 0x09, 0x03AC);
    SA868_WriteAT1846Sreg(_SerialRF, 0x24, 0x0001);
    SA868_WriteAT1846Sreg(_SerialRF, 0x31, 0x0031);
    SA868_WriteAT1846Sreg(_SerialRF, 0x33, 0x45F5);   // AGC number
    SA868_WriteAT1846Sreg(_SerialRF, 0x34, 0x2B89);   // RX digital gain
    SA868_WriteAT1846Sreg(_SerialRF, 0x3F, 0x3263);   // RSSI 3 threshold
    SA868_WriteAT1846Sreg(_SerialRF, 0x41, 0x470F);   // Tx digital gain
    SA868_WriteAT1846Sreg(_SerialRF, 0x42, 0x1036);
    SA868_WriteAT1846Sreg(_SerialRF, 0x43, 0x00BB);

    SA868_WriteAT1846Sreg(_SerialRF, 0x44, 0x06FF);   // Tx digital gain

    SA868_WriteAT1846Sreg(_SerialRF, 0x47, 0x7F2F);   // Soft mute
    SA868_WriteAT1846Sreg(_SerialRF, 0x4E, 0x0082);
    SA868_WriteAT1846Sreg(_SerialRF, 0x4F, 0x2C62);
    SA868_WriteAT1846Sreg(_SerialRF, 0x53, 0x0094);
    SA868_WriteAT1846Sreg(_SerialRF, 0x54, 0x2A3C);
    SA868_WriteAT1846Sreg(_SerialRF, 0x55, 0x0081);
    SA868_WriteAT1846Sreg(_SerialRF, 0x56, 0x0B02);
    SA868_WriteAT1846Sreg(_SerialRF, 0x57, 0x1C00);   // Bypass RSSI low-pass
    SA868_WriteAT1846Sreg(_SerialRF, 0x5A, 0x4935);   // SQ detection time
    SA868_WriteAT1846Sreg(_SerialRF, 0x58, 0xBCCD);
    SA868_WriteAT1846Sreg(_SerialRF, 0x62, 0x3263);   // Modulation detect tresh
    SA868_WriteAT1846Sreg(_SerialRF, 0x4E, 0x2082);
    SA868_WriteAT1846Sreg(_SerialRF, 0x63, 0x16AD);
    SA868_WriteAT1846Sreg(_SerialRF, 0x30, 0x40A4);
    delay(50);

    SA868_WriteAT1846Sreg(_SerialRF, 0x30, 0x40A6);   // Start calibration
    delay(100);
    SA868_WriteAT1846Sreg(_SerialRF, 0x30, 0x4006);   // Stop calibration

    delay(100);

    SA868_WriteAT1846Sreg(_SerialRF, 0x58, 0xBCED);
    SA868_WriteAT1846Sreg(_SerialRF, 0x0A, 0x7BA0);   // PGA gain
    SA868_WriteAT1846Sreg(_SerialRF, 0x41, 0x4731);   // Tx digital gain
    SA868_WriteAT1846Sreg(_SerialRF, 0x44, 0x05FF);   // Tx digital gain
    SA868_WriteAT1846Sreg(_SerialRF, 0x59, 0x09D2);   // Mixer gain
    SA868_WriteAT1846Sreg(_SerialRF, 0x44, 0x05CF);   // Tx digital gain
    SA868_WriteAT1846Sreg(_SerialRF, 0x44, 0x05CC);   // Tx digital gain
    SA868_WriteAT1846Sreg(_SerialRF, 0x48, 0x1A32);   // Noise 1 threshold
    SA868_WriteAT1846Sreg(_SerialRF, 0x60, 0x1A32);   // Noise 2 threshold
    SA868_WriteAT1846Sreg(_SerialRF, 0x3F, 0x29D1);   // RSSI 3 threshold
    SA868_WriteAT1846Sreg(_SerialRF, 0x0A, 0x7BA0);   // PGA gain
    SA868_WriteAT1846Sreg(_SerialRF, 0x33, 0x45F5);   // AGC number
    SA868_WriteAT1846Sreg(_SerialRF, 0x41, 0x470F);   // Tx digital gain
    SA868_WriteAT1846Sreg(_SerialRF, 0x42, 0x1036);
    SA868_WriteAT1846Sreg(_SerialRF, 0x43, 0x00BB);

    updateBandwidth();

    // FM mode
    SA868_WriteAT1846Sreg(_SerialRF, 0x33, 0x44A5);
    SA868_WriteAT1846Sreg(_SerialRF, 0x41, 0x4431);
    SA868_WriteAT1846Sreg(_SerialRF, 0x42, 0x10F0);
    SA868_WriteAT1846Sreg(_SerialRF, 0x43, 0x00A9);
    SA868_WriteAT1846Sreg(_SerialRF, 0x58, 0xBC05);   // Bit 0  = 1: CTCSS LPF badwidth to 250Hz
                                    // Bit 3  = 0: enable CTCSS HPF
                                    // Bit 4  = 0: enable CTCSS LPF
                                    // Bit 5  = 0: enable voice LPF
                                    // Bit 6  = 0: enable voice HPF
                                    // Bit 7  = 0: enable pre/de-emphasis
                                    // Bit 11 = 1: bypass VOX HPF
                                    // Bit 12 = 1: bypass VOX LPF
                                    // Bit 13 = 1: bypass RSSI LPF
    SA868_WriteAT1846Sreg(_SerialRF, 0x44, SA868_maskSetValue(0x06FF, 0x00F0, ((int16_t)_config.volume) << 8));
    SA868_WriteAT1846Sreg(_SerialRF, 0x40, 0x0030);

    SA868_maskSetRegister(_SerialRF, 0x57, 0x0001, 0x00);     // Audio feedback off
    SA868_maskSetRegister(_SerialRF, 0x3A, 0x7000, 0x4000);   // Select voice channel

    setSqlThresh();
    SA868_maskSetRegister(_SerialRF, 0x30, 0x0004, 0x0004);   // SQ ON
    setPower();
}

void OpenEdition::setBandwidth(uint8_t value)
{
    if (value > 1) {
        value = 0;
    }
    _config.band = value;
    updateBandwidth();
}

void OpenEdition::updateBandwidth()
{
    if (_config.band == 0) {
        // 12.5kHz bandwidth
        SA868_WriteAT1846Sreg(_SerialRF, 0x15, 0x1100);   // Tuning bit
        SA868_WriteAT1846Sreg(_SerialRF, 0x32, 0x4495);   // AGC target power
        SA868_WriteAT1846Sreg(_SerialRF, 0x3A, 0x4003);   // Modulation detect sel
        SA868_WriteAT1846Sreg(_SerialRF, 0x3F, 0x28D0);   // RSSI 3 threshold
        SA868_WriteAT1846Sreg(_SerialRF, 0x3C, 0x0F1E);   // Peak detect threshold
        SA868_WriteAT1846Sreg(_SerialRF, 0x48, 0x1DB6);   // Noise 1 threshold
        SA868_WriteAT1846Sreg(_SerialRF, 0x62, 0x1425);   // Modulation detect tresh
        SA868_WriteAT1846Sreg(_SerialRF, 0x65, 0x2494);
        SA868_WriteAT1846Sreg(_SerialRF, 0x66, 0xEB2E);   // RSSI comp and AFC range
        SA868_WriteAT1846Sreg(_SerialRF, 0x7F, 0x0001);   // Switch to page 1
        SA868_WriteAT1846Sreg(_SerialRF, 0x06, 0x0014);   // AGC gain table
        SA868_WriteAT1846Sreg(_SerialRF, 0x07, 0x020C);
        SA868_WriteAT1846Sreg(_SerialRF, 0x08, 0x0214);
        SA868_WriteAT1846Sreg(_SerialRF, 0x09, 0x030C);
        SA868_WriteAT1846Sreg(_SerialRF, 0x0A, 0x0314);
        SA868_WriteAT1846Sreg(_SerialRF, 0x0B, 0x0324);
        SA868_WriteAT1846Sreg(_SerialRF, 0x0C, 0x0344);
        SA868_WriteAT1846Sreg(_SerialRF, 0x0D, 0x1344);
        SA868_WriteAT1846Sreg(_SerialRF, 0x0E, 0x1B44);
        SA868_WriteAT1846Sreg(_SerialRF, 0x0F, 0x3F44);
        SA868_WriteAT1846Sreg(_SerialRF, 0x12, 0xE0EB);   // Back to page 0
        SA868_WriteAT1846Sreg(_SerialRF, 0x7F, 0x0000);
        SA868_maskSetRegister(_SerialRF, 0x30, 0x3000, 0x0000);
    } else {
        // 25kHz bandwidth
        SA868_WriteAT1846Sreg(_SerialRF, 0x15, 0x1F00);   // Tuning bit
        SA868_WriteAT1846Sreg(_SerialRF, 0x32, 0x7564);   // AGC target power
        SA868_WriteAT1846Sreg(_SerialRF, 0x3A, 0x4003);   // Modulation detect sel
        SA868_WriteAT1846Sreg(_SerialRF, 0x3F, 0x29D2);   // RSSI 3 threshold
        SA868_WriteAT1846Sreg(_SerialRF, 0x3C, 0x0E1C);   // Peak detect threshold
        SA868_WriteAT1846Sreg(_SerialRF, 0x48, 0x1E38);   // Noise 1 threshold
        SA868_WriteAT1846Sreg(_SerialRF, 0x62, 0x3767);   // Modulation detect tresh
        SA868_WriteAT1846Sreg(_SerialRF, 0x65, 0x248A);
        SA868_WriteAT1846Sreg(_SerialRF, 0x66, 0xFF2E);   // RSSI comp and AFC range
        SA868_WriteAT1846Sreg(_SerialRF, 0x7F, 0x0001);   // Switch to page 1
        SA868_WriteAT1846Sreg(_SerialRF, 0x06, 0x0024);   // AGC gain table
        SA868_WriteAT1846Sreg(_SerialRF, 0x07, 0x0214);
        SA868_WriteAT1846Sreg(_SerialRF, 0x08, 0x0224);
        SA868_WriteAT1846Sreg(_SerialRF, 0x09, 0x0314);
        SA868_WriteAT1846Sreg(_SerialRF, 0x0A, 0x0324);
        SA868_WriteAT1846Sreg(_SerialRF, 0x0B, 0x0344);
        SA868_WriteAT1846Sreg(_SerialRF, 0x0D, 0x1384);
        SA868_WriteAT1846Sreg(_SerialRF, 0x0E, 0x1B84);
        SA868_WriteAT1846Sreg(_SerialRF, 0x0F, 0x3F84);
        SA868_WriteAT1846Sreg(_SerialRF, 0x12, 0xE0EB);
        SA868_WriteAT1846Sreg(_SerialRF, 0x7F, 0x0000);   // Back to page 0
        SA868_maskSetRegister(_SerialRF, 0x30, 0x3000, 0x3000);
    }
}

void OpenEdition::setVolume(uint8_t value)
{
    _config.volume = value;
    SA868_maskSetRegister(_SerialRF, 0x44, 0x00F0, ((int16_t)_config.volume) << 8);
}

int16_t OpenEdition::getRSSI()
{
    return -137 + static_cast< int16_t >(SA868_ReadAT1846Sreg(_SerialRF, 0x1B) >> 8);
}

OpenEdition_Version OpenEdition::Version()
{
    OpenEdition_Version version;
    String data;
    if (!SA868_WaitResponse(_SerialRF, "AT+VERSION\r\n", &data))
    {
        ESP_LOGD("SA8x8", "Version Error");
        return version;
    }
    sscanf(data.c_str(), "sa8x8-fw/v%hhu.%hhu.%hhu.r%hhu", &version.major, &version.minor, &version.patch, &version.revision);
    ESP_LOGD("SA8x8", "Version %d.%d.%d.%d", version.major, version.minor, version.patch, version.revision);

    return version;
}

void OpenEdition::setFrequency(uint32_t freq)
{
    // AT1846S datasheet specifies a frequency step of 1/16th of kHz per bit.
    // Computation of register value is done using 64 bit to avoid overflows,
    // result is then truncated to 32 bits to fit it into the registers.
    uint64_t val = ((uint64_t) freq * 16) / 1000;
    val &= 0xFFFFFFFF;

    uint16_t fHi = (val >> 16) & 0xFFFF;
    uint16_t fLo = val & 0xFFFF;

    SA868_WriteAT1846Sreg(_SerialRF, 0x29, fHi);
    SA868_WriteAT1846Sreg(_SerialRF, 0x2A, fLo);

    SA868_reloadConfig(_SerialRF);
}

void OpenEdition::setSqlThresh(uint8_t value)
{
    _config.sql_level = value;
    setSqlThresh();
}

void OpenEdition::setSqlThresh()
{
    SA868_WriteAT1846Sreg(_SerialRF, 0x49, static_cast< uint16_t >(_config.sql_level));
    SA868_WriteAT1846Sreg(_SerialRF, 0x48, static_cast< uint16_t >(_config.sql_level));
}

void OpenEdition::RxOn()
{
    if (_config.mode == OpenEdition_Mode::RX)
        return;
    OpenEdition::setFrequency(_config.freq_rx);
    SA868_setFuncMode(_SerialRF, OpenEdition_Mode::RX);
    _config.mode = OpenEdition_Mode::RX;
}

void OpenEdition::TxOn()
{
    if (_config.mode == OpenEdition_Mode::TX)
        return;
    OpenEdition::setFrequency(_config.freq_tx);
    SA868_setFuncMode(_SerialRF, OpenEdition_Mode::TX);
    _config.mode = OpenEdition_Mode::TX;
}

void OpenEdition::TxOff()
{
    if (_config.mode != OpenEdition_Mode::TX)
        return;
    SA868_setFuncMode(_SerialRF, OpenEdition_Mode::OFF);
    _config.mode = OpenEdition_Mode::OFF;
}

OpenEdition_Configuration OpenEdition::settings()
{
    return _config;
}

bool OpenEdition::isHighPower()
{
    return _config.rf_power;
}

void OpenEdition::setHighPower()
{
    _config.rf_power = true;
    OpenEdition::setPower();
}

void OpenEdition::setLowPower()
{
    _config.rf_power = false;
    OpenEdition::setPower();
}

void OpenEdition::setPower()
{
    char str[20];
    String result;
    sprintf(str, "AT+AMP=%d\r\n", _config.rf_power);
    if (!SA868_WaitResponse(_SerialRF, str, &result))
    {
        ESP_LOGD("SA8x8", "can't enable power amplifier");
    }
}
