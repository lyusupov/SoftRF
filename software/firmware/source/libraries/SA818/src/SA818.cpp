#include <Arduino.h>
#include <SA818.h>

SA818::SA818(Stream* stream) : stream_(stream) {}

SA818::~SA818() {
    stream_ = 0;
    lastResponse_.req = 0;
}

void SA818::verbose() {
    debug_ = true;
}

void SA818::setTimeout(long timeout) {
    timeout_ = timeout;
}

void SA818::debug(String str) {
    if(Serial && debug_) {
        Serial.print(str);
    }
}

bool SA818::send(char* cmd, int size, char** args) {
    lastResponse_.req = 0;

    Request req;
    req.req = cmd;
    req.size = size;
    req.args = args;

    bool ok = request(&req);
    return ok;
}

bool SA818::request(Request* req) {
    debug("-> ");

    print(req->req);
    
    if(req->size > 0) 
        print("=");

    for(int i = 0; i < req->size; i++) {
        print(req->args[i]);
        if(i < req->size - 1) 
            print(",");
    }
    flush();

    Response resp;
    resp.req = req;
    resp.complete = false;

    waitResponse(&resp);

    debug("<- " + resp.raw + "\n");

    bool ok = success(&resp);
    return ok;
}

void SA818::print(char* str) {
    debug(str);
    stream_->print(str);
}

void SA818::flush() {
    debug("\n");
    stream_->print("\n\r\n");
}

bool SA818::readByte(byte* tk) {
    if(stream_->available() > 0) {
        *tk = stream_->read(); // Warning : the buffer is emptied
        return *tk != -1;
    }
    return false;
}

bool SA818::waitResponse(Response* resp) {
    String raw = "";
    String res = "";

    bool loop = true;
    long timer = millis();
    long time;
    byte zone = 0;
    do {
        byte tk = -1;
        if(readByte(&tk)) {
            if(tk != 13 && tk != 10) { // not [\r] or [\n]
                raw += (char) tk;

                if(zone == 1) {
                     res += (char) tk;
                }

                if(tk == 58 || tk == 61) { // [:] or [=]
                   zone++;
                }
            }

            if(tk == 10) { // [\n]
                loop = false;
                resp->complete = true;
            }
            else
                timer = millis();
        }

        time = millis() - timer;
        loop = time < timeout_;

    } while(loop);
    
    resp->raw = raw;
    resp->res = res;

    return resp->complete;
}

bool SA818::success(Response* resp) {
    if(resp->complete) {
        lastResponse_ = *resp;

        if(resp->res != "") {
            if(resp->res.equals("1")) return false;
            return true;
        }

        if(resp->raw.equals("+DMOERROR")) return false;
    }

    return false;
}

Response* SA818::response() {
    return &lastResponse_;
}
