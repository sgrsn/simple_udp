#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <cstring>

class SimpleUdp
{
private:
  int socket_;
  struct sockaddr_in addr_;
public:
  SimpleUdp(std::string address, int port, int socket_type=SOCK_DGRAM)
  {
    if ((socket_ = socket(AF_INET, socket_type, IPPROTO_UDP)) < 0){
      return;
    }
    addr_.sin_family = AF_INET;
    addr_.sin_addr.s_addr = inet_addr(address.c_str());
    addr_.sin_port = htons(port);        
  }
  template<typename T>
  void udp_send(T data){
    sendto(socket_, &data, sizeof(T), 0, (struct sockaddr *)&addr_, sizeof(addr_));
  }
  void udp_bind(){
    bind(socket_, (const struct sockaddr *)&addr_, sizeof(addr_));
  }
  std::string udp_recv(){
    #define BUFFER_MAX 400
    char buf[BUFFER_MAX] = {0};
    recv(socket_, buf, sizeof(buf), 0);
    return std::string(buf);
  }
  void udp_recv(char *buf, int size){
    memset(buf, 0, size);
    recv(socket_, buf, size, 0);
  }
  ~SimpleUdp(){
    close(socket_);
  }
};

template <>
void SimpleUdp::udp_send<std::string>(std::string word){
  sendto(socket_, word.c_str(), word.length(), 0, (struct sockaddr *)&addr_, sizeof(addr_));
}