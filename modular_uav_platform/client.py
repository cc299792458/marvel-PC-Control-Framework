#_*_ coding:utf-8 _*_
#客户端
#导入socket模块
import socket

if __name__ == '__main__':
    #创建TCP类型的socket
    c = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #建立到指定IP地址,端口的TCP连接
    c.connect(('10.1.125.223', 80))
    while 1:
        data = input("please go out:")
        c.send(bytes(data, 'utf-8'))
        # result = c.recv(1024)
        # print(result) #输出接收到的结果
    c.close()
