#ifndef SA818_H
#define SA818_H

class Request {
    public:
        char* req;
        int size;
        char** args;
};

class Response {
    public:
        Request* req;
        bool complete;
        String raw;
        String res;
};

enum Model { SA_818 = 0, SA_868_NiceRF = 1, SA_868_OpenEdition = 2 };
enum Band { UHF = 0, VHF = 1 }; //400-480MHz // 134-174MHz

class SA818 {
    private:
        bool debug_ = false;
        Stream* stream_ = 0;
        long timeout_ = 1000;

        Response lastResponse_;

        bool request(Request*);
        void print(char*);
        void flush();
        void debug(String);

        bool readByte(byte*);
        bool waitResponse(Response*);
        bool success(Response*);

    public:
        SA818(Stream*);
        ~SA818();

        void verbose();
        void setTimeout(long = 1000);

        bool send(char*, int = 0, char** = 0);
        Response* response();
};

#endif
