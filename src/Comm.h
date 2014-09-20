#include <stdio.h>
#include <sys/termios.h>
#include <cstdlib>
#include <cstring>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>

// CComm

#define DEFAULT_BAUDRATE 19200

/** シリアルポートを開き、通信を行うクラス
 */
class CComm
{
private:
	int com_port;
	int fd;

public:
	CComm();
	virtual ~CComm();

	/** 設定用のビットレートを出力する
	* @param[in] baudrate ビットレート
	* @return bitrate
	*/
	speed_t Bitrate(int aBR);

	//! シリアルポートの通信パラメータを設定する
	/*!
	  aFd で指定されたポートの通信パラメータを設定する。
	  通信速度を aSpeed に設定するほか、受信タイムアウトを 500ms に設定する。
	  @param [in] aFd シリアルポートを指すファイルディスクリプタ
	  @param [in] aSpeed 設定する通信速度 (bps)
	  @retval false 失敗
	  @retval true 成功
	 */
	bool setupComm(int aFd, int aSpeed);

	/** COMポートを開く
	* @param[in] port ポート番号
	* @param[in] baudrate ビットレート
	* @return 成功の場合は真
	*/
	bool Open(char* port, int baudrate = DEFAULT_BAUDRATE);

	/** COMポートを閉じる
	 * @return 成功の場合は真
	 */
	bool Close(void);

	/** データを送信する
	 * @param[in] data 送信するデータ
	 * @param[in] len データの長さ
	 * @return 送信したバイト数
	 */
	int Send(char *data, int len = 0);

	/** データを送信する
	 * @param[in] data 受信用バッファ
	 * @param[in] len バッファサイズ
	 * @return 受信したバイト数
	 */
	int Recv(char *data, int max_len);

	/** 未受信のデータのクリア
	 * @return 成功の場合は真
	 */
	bool ClearRecvBuf(void);
};


