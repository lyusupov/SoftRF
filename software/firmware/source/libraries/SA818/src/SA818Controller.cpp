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
