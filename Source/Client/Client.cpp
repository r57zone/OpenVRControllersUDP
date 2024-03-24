#include <iostream>
#include <winsock2.h>
#include <Ws2tcpip.h>
#pragma comment (lib, "WSock32.Lib")
#pragma warning(disable:4996) 

SOCKET socketS;
int bytes_read;
bool SocketActivated = false;
bool bKeepReading = false;

typedef struct _Controller
{
	double	X;
	double	Y;
	double	Z;
	double	QuatW;
	double	QuatX;
	double	QuatY;
	double	QuatZ;
	unsigned short	Buttons;
	float	Trigger;
	float	AxisX;
	float	AxisY;
} TController, *PController;

#define GRIP_BTN	0x0001
#define THUMB_BTN	0x0002
#define MENU_BTN	0x0010
#define SYS_BTN		0x0020

TController MyCtrl, MyCtrl2;

bool CtrlsConnected = false;

typedef struct _Controllers
{
	TController Controller1;
	TController Controller2;
} TControllers, *PControllers;

TControllers MyControllers;

int main()
{
	SetConsoleTitle("Remote console of controllers");

	//std::cout << "Client started!\n";

	WSADATA wsaData;
	int iResult;
	sockaddr_in local;

	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult == 0) {
		local.sin_family = AF_INET;
		local.sin_port = htons(4243);
		local.sin_addr.s_addr = inet_addr("127.0.0.1");

		socketS = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

		iResult = connect(socketS, (sockaddr *)&local, sizeof(local));
		SocketActivated = true;
		printf("OpenVR remote\n");
	}
	else
	{
		WSACleanup();
		SocketActivated = false;
	}


	memset(&MyControllers, 0, sizeof(MyControllers));

	MyControllers.Controller1.Y = -0.5;
	
	MyControllers.Controller2.Y = -0.5;
	MyControllers.Controller2.X = -0.2;
	MyControllers.Controller1.Z = -0.2;
	
	while (SocketActivated) {
		MyControllers.Controller1.Buttons = 0;
		MyControllers.Controller2.Buttons = 0;

		if ((GetAsyncKeyState(VK_ESCAPE) & 0x8000) != 0) break;

		if ((GetAsyncKeyState(VK_LEFT) & 0x8000) != 0) MyControllers.Controller1.X -= 0.005;
		if ((GetAsyncKeyState(VK_RIGHT) & 0x8000) != 0) MyControllers.Controller1.X += 0.005;

		if ((GetAsyncKeyState(VK_DELETE) & 0x8000) != 0) MyControllers.Controller1.Z -= 0.005;
		if ((GetAsyncKeyState(VK_INSERT) & 0x8000) != 0) MyControllers.Controller1.Z += 0.005;

		if ((GetAsyncKeyState(VK_DOWN) & 0x8000) != 0) MyControllers.Controller1.Y += 0.005;
		if ((GetAsyncKeyState(VK_UP) & 0x8000) != 0) MyControllers.Controller1.Y -= 0.005;

		MyControllers.Controller2.X = MyControllers.Controller1.X + 0.2;
		MyControllers.Controller2.Y = MyControllers.Controller1.Y;
		MyControllers.Controller2.Z = MyControllers.Controller1.Z;
		
		if ((GetAsyncKeyState(VK_NUMPAD0) & 0x8000) != 0)
			MyControllers.Controller1.Trigger = 1.0f;
		if ((GetAsyncKeyState(VK_DECIMAL) & 0x8000) != 0)
			MyControllers.Controller1.Buttons |= GRIP_BTN;
		if ((GetAsyncKeyState(VK_NUMPAD1) & 0x8000) != 0)
			MyControllers.Controller1.Buttons |= MENU_BTN;
		if ((GetAsyncKeyState(VK_NUMPAD2) & 0x8000) != 0)
			MyControllers.Controller1.Buttons |= SYS_BTN;

		iResult = send(socketS, (char*)(&MyControllers), sizeof(MyControllers), 0);
		if (iResult == SOCKET_ERROR) {
			printf("Connection lost\n");
			break;
		}

		Sleep(1);
	}


	if (SocketActivated) {
		SocketActivated = false;
		closesocket(socketS);
		WSACleanup();
	}

}
