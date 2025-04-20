#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace at
{
class Keyboard
{
public:
  Keyboard()
  {
    tcgetattr(STDIN_FILENO, &orig_termios);
    raw_termios = orig_termios;

    // 修改输入模式标志 - 移除 ISIG 的禁用
    raw_termios.c_lflag &= ~(ICANON | ECHO | IEXTEN);

    // 修改输入标志
    raw_termios.c_iflag &= ~(IXON | IXOFF | ICRNL | INLCR | IGNCR);

    // 设置读取模式
    raw_termios.c_cc[VMIN] = 0;   // 最小字符数
    raw_termios.c_cc[VTIME] = 0;  // 等待时间

    tcsetattr(STDIN_FILENO, TCSANOW, &raw_termios);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);  // 非阻塞读取
  }

  ~Keyboard() { tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios); }

  char get_char()
  {
    char ch;
    if (read(STDIN_FILENO, &ch, 1) == 1) return ch;
    return -1;
  }

private:
  termios orig_termios;
  termios raw_termios;
};
}  // namespace at
