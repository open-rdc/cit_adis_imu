// Comm.cpp : 実装ファイル
// 09/1/13 清水 Linuxに移植する

#include "Comm.h"

//OVERLAPPED sendop, recop;

// CComm

CComm::CComm(const std::string com_port, int baudrate = DEFAULT_BAUDRATE) {
	com_port_ = com_port;
	baudrate_ = baudrate;
}

CComm::~CComm() {
}

speed_t CComm::Bitrate() {
	switch (baudrate_) {
	case 9600:
		return B9600;

	case 19200:
		return B19200;

	case 38400:
		return B38400;

	case 57600:
		return B57600;

	case 115200:
		return B115200;

	case 230400:
		return B230400;
#if 0
		case 460800:
		return B460800;

		case 500000:
		return B500000;
#endif
	default:
		return B0;
	}
}

bool CComm::setupComm(int aFd){
	int speed;
	struct termios tio;

	speed=Bitrate();
	if(speed==0)
		return false;  // invalid bitrate

	tcgetattr(aFd,&tio);

	//  cfsetispeed(&tio,speed); // bitrate
	cfsetospeed(&tio,speed); // bitrate

//	cfmakeraw(&tio); // raw mode
/*
	tio.c_iflag = 0;
	tio.c_oflag = 0;
	tio.c_cflag &= ~(CSIZE | PARENB | CSTOPB);
	tio.c_cflag |= CS8 | CREAD | CLOCAL;
	tio.c_lflag &= ~(ICANON | ECHO | ISIG | IEXTEN);
*/
	tio.c_cflag=(tio.c_cflag & ~CSIZE) | CS8; // data bits = 8bit

	tio.c_iflag&= ~( BRKINT | ICRNL | ISTRIP );
	tio.c_iflag&= ~ IXON;    // no XON/XOFF
	tio.c_cflag&= ~PARENB;   // no parity
	tio.c_cflag&= ~CRTSCTS;  // no CTS/RTS
	tio.c_cflag&= ~CSTOPB;   // stop bit = 1bit
	tio.c_cc[VTIME] = 10;    // 100ms to timeout
	tio.c_cc[VMIN] =10;


	// Other
	tio.c_lflag &= ~( ISIG | ICANON | ECHO );// | CIGNORE);

	// Commit
	if(tcsetattr(aFd,TCSADRAIN,&tio)==0)
		return true;
	return false;
}

// 通信開始時には，必ず実行する
bool CComm::Open() {
	fd=open(com_port_.c_str(),O_RDWR | O_NOCTTY);
	
	std::cout << fd << std::endl;

	if(fd < 0){
		std::cerr << "Can not Open " << com_port_ << "." << std::endl;
		return false;  // invalid device file
	}

	// setup parameters
	if(setupComm(fd)){
		return true; // 正常終了
	}
	else {
		perror("setting term parameters");
		close(fd);
		return false;
	}
}

// 通信終了時には，必ず実行する
bool CComm::Close(void) {
	close(fd);
	return true;
}

// データをlenだけ送信する．
// 戻り値: lenと同じ数の時は正常終了，-1:エラー
int CComm::Send(char *data, int len) {
	unsigned short retlen;

	if (fd < 0)
		return -1; // ハンドルがない場合
	if (len == 0) { // もし文字数を省略したらNULLまでの文字数
		len = (int) strlen(data);
	}
	const char *p = data;
	const char * const endp = p + len;
	while(p < endp) {
		int num_bytes = write(fd, p, endp - p);
		if (num_bytes < 0)
			break;
		p+=num_bytes;
	}

	//retlen = write(fd, data, len);
	return retlen;
}

// max_lenバイト以下のデータを受信する
// 戻り値: 受信した文字数，-1:エラー
int CComm::Recv(char *data, int max_len) {
	int len;
	fd_set mask;
	fd_set readOK;
	struct timeval blocktime;
	blocktime.tv_sec = 1;
	blocktime.tv_usec = 0;

	if (fd < 0)
		return -1; // ハンドルがない場合

	FD_ZERO(&mask); // ディスクリプタ監視用ビットをすべて0にして初期化
	FD_ZERO(&readOK);
	FD_SET(fd, &readOK); // 監視するディスクリプタに該当するビットを立てる
	select(fd+1, &readOK, NULL, NULL, &blocktime); // ディスクリプタを監視。
	if(!FD_ISSET(fd, &readOK))  // 指定したディスクリプタにデータが届いていれば真
		return 0; // 受信していない場合

	len = read(fd, data, max_len);

	return len;
}

bool CComm::ClearRecvBuf(void) {
	const int buf_size = 1000;
	//const int buf_size = 1500;
	char buf[buf_size];

	while (Recv(buf, buf_size) > 0)
		;
	return true;
}

