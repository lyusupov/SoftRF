#include <iostream>
#include "TCPServer.h"

TCPServer tcp;

void * loop(void * m)
{
	pthread_detach(pthread_self());
	while(1)
	{
		srand(time(NULL));
		char ch = 'a' + rand() % 26;
		string s(1,ch);
		string str = tcp.getMessage();
		if( str != "" )
		{
			cout << "Message:" << str << endl;
			tcp.Send(" [client message: "+str+"] "+s);
			tcp.clean();
		}
		usleep(1000);
	}
	tcp.detach();
}

int main()
{
	pthread_t msg;
	tcp.setup(11999);
	if( pthread_create(&msg, NULL, loop, (void *)0) == 0)
	{
		tcp.receive();
	}
	return 0;
}
