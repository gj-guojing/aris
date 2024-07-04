/// \example demo_object/main.cpp
/// 本例子展示构造数据结构的过程:
///

#include "aris.hpp"

auto test1()->void {
	std::cout << "-----------------------------test1-------------------------------" << std::endl;

	aris::core::SocketServer server;
	aris::core::Socket client;
	server.setConnectType(aris::core::SocketServer::Type::TCP_RAW);
	client.setConnectType(aris::core::Socket::Type::TCP_RAW);


	server.setOnReceivedRawData([](aris::core::SocketServer* s, aris::core::SOCKET_T sock, const char* data, int size)->int {
		std::cout << "server received:" << std::string(data, size) << std::endl;
		return 0;
		});

	client.setOnReceivedRawData([](aris::core::Socket* s, const char* data, int size)->int {
		std::cout << "client received:" << std::string(data, size) << std::endl;
		return 0;
		});


	try {
		server.startServer("5866");
	}
	catch (std::exception &e) {
		std::cout << e.what() << std::endl;
	
	}
	try {
		server.startServer("5866");
	}
	catch (std::exception &e) {
		std::cout << e.what() << std::endl;

	}
	
	client.connect("127.0.0.1", "5866");

	client.sendRawData("abcd\n", 5);
	client.sendRawData("abcde\n", 6);
	client.sendRawData("abcdef\n", 7);

	std::this_thread::sleep_for(std::chrono::seconds(1));

	//server.sendRawData(server.remoteIpMap().begin()->first, "ABCDEFG\n", 8);
	//server.sendRawData(server.remoteIpMap().begin()->first, "ABCDEF\n", 7);
	//server.sendRawData(server.remoteIpMap().begin()->first, "ABCDE\n", 6);

	std::this_thread::sleep_for(std::chrono::seconds(1));

	server.stop();
	client.stop();
	std::this_thread::sleep_for(std::chrono::seconds(1));
	std::cout << "-----------------------------test1-------------------------------" << std::endl;
}
auto test2() -> void {
	std::cout << "-----------------------------test2-------------------------------" << std::endl;

	aris::core::SocketServer server;
	aris::core::Socket client, client2;
	server.setConnectType(aris::core::SocketServer::Type::TCP_RAW);
	client.setConnectType(aris::core::Socket::Type::TCP_RAW);
	client2.setConnectType(aris::core::Socket::Type::TCP_RAW);

	server.setOnReceivedRawData([](aris::core::SocketServer* s, aris::core::SOCKET_T sock, const char* data, int size)->int {
		std::cout << "server received " << sock << " : " << std::string(data, size) << std::endl;
		return 0;
		});

	client.setOnReceivedRawData([](aris::core::Socket* s, const char* data, int size)->int {
		std::cout << "client received:" << std::string(data, size) << std::endl;
		return 0;
		});

	server.startServer("5866");
	client.connect("127.0.0.1", "5866");
	client2.connect("127.0.0.1", "5866");

	client.sendRawData("abcd\n", 5);
	client.sendRawData("abcde\n", 6);
	client.sendRawData("abcdef\n", 7);
	client2.sendRawData("abcd\n", 5);
	client2.sendRawData("abcde\n", 6);
	client2.sendRawData("abcdef\n", 7);


	std::this_thread::sleep_for(std::chrono::seconds(1));

	client2.sendRawData("ds\n", 5);
	client2.sendRawData("aaa\n", 6);
	client2.sendRawData("sqwe\n", 7);

	client.sendRawData("23213\n", 5);
	client.sendRawData("123\n", 6);
	client.sendRawData("43\n", 7);

	//server.sendRawData(server.remoteIpMap().begin()->first, "ABCDEFG\n", 8);
	//server.sendRawData(server.remoteIpMap().begin()->first, "ABCDEF\n", 7);
	//server.sendRawData(server.remoteIpMap().begin()->first, "ABCDE\n", 6);

	client.stop();
	std::this_thread::sleep_for(std::chrono::seconds(1));
	std::cout << "-----------------------------test2-------------------------------" << std::endl;
}
auto test3() -> void {
	std::cout << "-----------------------------test3-------------------------------" << std::endl;

	aris::core::SocketServer server;
	aris::core::Socket client, client2;
	server.setConnectType(aris::core::SocketServer::Type::TCP);
	client.setConnectType(aris::core::Socket::Type::TCP);
	client2.setConnectType(aris::core::Socket::Type::TCP);

	server.setOnReceivedMsg([](aris::core::SocketServer* s, aris::core::SOCKET_T sock, aris::core::Msg &msg)->int {
		ARIS_COUT << "server received " << sock << " : " << msg.toString() << std::endl;
		
		static int i = 0;
		++i;
		s->sendMsg(sock, aris::core::Msg("reply:" + std::to_string(i)));
		return 0;
		});

	client.setOnReceivedMsg([](aris::core::Socket* s, aris::core::Msg& msg)->int {
		ARIS_COUT << "client received:" << msg.toString() << std::endl;
		return 0;
		});


	

	for (int i = 0; i < 10000; ++i) {
		server.startServer("5866");
		client.connect("127.0.0.1", "5866");
		client2.connect("127.0.0.1", "5866");

		{
			aris::core::Msg msg;
			
			msg.copy("123435");
			msg.resize(2048);
			client.sendMsg(msg);
		}

		client.sendMsg(aris::core::Msg("111ds\n"));
		client.sendMsg(aris::core::Msg("111dssdsa\n"));
		client.sendMsg(aris::core::Msg("222ds213\n"));
		client2.sendMsg(aris::core::Msg("ds\n"));
		client2.sendMsg(aris::core::Msg("dssdsa\n"));
		client2.sendMsg(aris::core::Msg("ds213\n"));


		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		client2.sendMsg(aris::core::Msg("ds\n"));
		client2.sendMsg(aris::core::Msg("dssdsa\n"));
		client2.sendMsg(aris::core::Msg("ds213\n"));
		client.sendMsg(aris::core::Msg("111ds\n"));
		client.sendMsg(aris::core::Msg("111dssdsa\n"));
		client.sendMsg(aris::core::Msg("222ds213\n"));

		server.stop();
	}


	//server.sendRawData(server.remoteIpMap().begin()->first, "ABCDEFG\n", 8);
	//server.sendRawData(server.remoteIpMap().begin()->first, "ABCDEF\n", 7);
	//server.sendRawData(server.remoteIpMap().begin()->first, "ABCDE\n", 6);

	std::this_thread::sleep_for(std::chrono::seconds(1));

	client.stop();
	std::this_thread::sleep_for(std::chrono::seconds(1));
	std::cout << "-----------------------------test3-------------------------------" << std::endl;
}
auto test4() -> void {
	std::cout << "-----------------------------test4-------------------------------" << std::endl;

	aris::core::SocketServer server;
	aris::core::Socket client, client2;
	server.setConnectType(aris::core::SocketServer::Type::WEB);
	client.setConnectType(aris::core::Socket::Type::WEB);
	client2.setConnectType(aris::core::Socket::Type::WEB);

	server.setOnReceivedMsg([](aris::core::SocketServer* s, aris::core::SOCKET_T sock, aris::core::Msg& msg)->int {
		ARIS_COUT << "server received " << sock << " : " << msg.toString() << std::endl;

		static int i = 0;
		++i;
		s->sendMsg(sock, aris::core::Msg("reply:" + std::to_string(i)));
		return 0;
		});

	client.setOnReceivedMsg([](aris::core::Socket* s, aris::core::Msg& msg)->int {
		ARIS_COUT << "client received:" << msg.toString() << std::endl;
		return 0;
		});




	for (int i = 0; i < 10000; ++i) {
		server.startServer("5866");
		client.connect("127.0.0.1", "5866");
		client2.connect("127.0.0.1", "5866");

		{
			aris::core::Msg msg;

			msg.copy("123435");
			msg.resize(2048);
			client.sendMsg(msg);
		}

		client.sendMsg(aris::core::Msg("111ds\n"));
		client.sendMsg(aris::core::Msg("111dssdsa\n"));
		client.sendMsg(aris::core::Msg("222ds213\n"));
		client2.sendMsg(aris::core::Msg("ds\n"));
		client2.sendMsg(aris::core::Msg("dssdsa\n"));
		client2.sendMsg(aris::core::Msg("ds213\n"));


		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		client2.sendMsg(aris::core::Msg("ds\n"));
		client2.sendMsg(aris::core::Msg("dssdsa\n"));
		client2.sendMsg(aris::core::Msg("ds213\n"));
		client.sendMsg(aris::core::Msg("111ds\n"));
		client.sendMsg(aris::core::Msg("111dssdsa\n"));
		client.sendMsg(aris::core::Msg("222ds213\n"));

		client.stop();
		client2.stop();
	}


	//server.sendRawData(server.remoteIpMap().begin()->first, "ABCDEFG\n", 8);
	//server.sendRawData(server.remoteIpMap().begin()->first, "ABCDEF\n", 7);
	//server.sendRawData(server.remoteIpMap().begin()->first, "ABCDE\n", 6);

	std::this_thread::sleep_for(std::chrono::seconds(1));

	client.stop();
	std::this_thread::sleep_for(std::chrono::seconds(1));
	std::cout << "-----------------------------test3-------------------------------" << std::endl;
}

#define SERVER_SOCKET aris::core::SocketServer
#define CLIENT_SOCKET aris::core::Socket
#define REMOTE_IP "127.0.0.1"
#define PORT "5866"
auto test5() {
	SERVER_SOCKET server("server");
	server.setConnectType(SERVER_SOCKET::Type::WEB);

	server.setOnReceivedConnection([](SERVER_SOCKET* server, aris::core::SOCKET_T sock, const char* remote_ip, int port)->int {
		std::cout << "server receive connnection from " + std::string(remote_ip) + " : " + std::to_string(port) << std::endl;
		return 0;
		});
	server.setOnLoseConnection([](SERVER_SOCKET* server, aris::core::SOCKET_T sock)->int {
		std::cout << "server lose connection" << std::endl;
		return 0;
		});
	server.setOnReceivedRawData([](SERVER_SOCKET* server, aris::core::SOCKET_T sock, const char* data, int size)->int {
		std::cout << "server recv " + std::to_string(size) + " bytes raw data " +/*"from "+sock->remoteIP()+":"+sock->port()+*/" -- " + std::string(data, size) << std::endl;
		return 0;
		});
	server.setOnReceivedMsg([](SERVER_SOCKET* server, aris::core::SOCKET_T sock, aris::core::Msg& msg)->int {
		std::cout << "server recv " + std::to_string(msg.size()) + " bytes msg " +/*"from "+sock->remoteIP()+":"+sock->port()+*/" -- " + msg.toString() << std::endl;
		// server->sendMsg(sock, aris::core::Msg(std::to_string(2*std::stoi(msg.toString()))));
		return 0;
		});

	// SERVER_SOCKET server("server");

	// server.setOnReceivedConnection([](SERVER_SOCKET *server, const char *remote_ip, int port)->int {
	// 	std::cout << "server receive connnection from "+std::string(remote_ip)+" : "+std::to_string(port) << std::endl;
	// 	return 0;
	// });
	// server.setOnLoseConnection([](SERVER_SOCKET *server)->int {
	// 	std::cout << "server lose connection" << std::endl;
	// 	return 0;
	// });
	// server.setOnReceivedRawData([](SERVER_SOCKET *server, const char *data, int size)->int {
	// 	std::cout << "server recv "+std::to_string(size)+" bytes raw data "+/*"from "+sock->remoteIP()+":"+sock->port()+*/" -- " + std::string(data, size) << std::endl;
	// 	return 0;
	// });
	// server.setOnReceivedMsg([](aris::core::SOCKET_T sock, aris::core::Msg &msg)->int {
	// 	std::cout << "server recv "+std::to_string(msg.size())+" bytes msg "+/*"from "+sock->remoteIP()+":"+sock->port()+*/" -- " + msg.toString() << std::endl;
	// 	// server->sendMsg(aris::core::Msg(std::to_string(2*std::stoi(msg.toString()))));
	// 	return 0;
	// });

	server.startServer(PORT);

	constexpr int num_clients{ 1 }, num_send{ 6 };
	std::thread th[num_clients];
	for (int i = 0; i < num_clients; ++i) {
		th[i] = std::thread([i, num_send]()->void {
			const std::string& cname = "client-" + std::to_string(i);
			CLIENT_SOCKET client(cname, REMOTE_IP, PORT);
			client.setConnectType(CLIENT_SOCKET::Type::WEB);
			client.setOnReceivedConnection([cname](CLIENT_SOCKET* sock, const char* remote_ip, int port)->int {
				std::cout << cname + " is connected" << std::endl;
				return 0;
				});
			client.setOnLoseConnection([cname](CLIENT_SOCKET* sock)->int {
				std::cout << cname + " lose connection" << std::endl;
				return 0;
				});
			client.setOnReceivedRawData([cname](CLIENT_SOCKET* sock, const char* data, int size)->int {
				std::cout << cname + " recv " + std::to_string(size) + " bytes raw data " + "from " + sock->remoteIP() + ":" + sock->port() + " -- " + std::string(data, size) << std::endl;
				return 0;
				});
			client.setOnReceivedMsg([i, cname](CLIENT_SOCKET* sock, aris::core::Msg& msg)->int {
				std::cout << cname + " recv " + std::to_string(msg.size()) + " bytes msg " + "from " + sock->remoteIP() + ":" + sock->port() + " -- " + msg.toString() << std::endl;
				assert(2 * i == std::stoi(msg.toString()));
				return 0;
				});

			client.setConnectTimeoutMs(1000000);
			client.connect(REMOTE_IP, PORT);

			for (int j = 0; j < num_send / 2; ++j) {
				client.sendMsg(aris::core::Msg(std::to_string(i)));
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}


			client.stop();
			std::cout << "before connect" << std::endl;
			client.connect();
			std::cout << "after connect" << std::endl;

			for (int j = 0; j < num_send / 2; ++j) {
				client.sendMsg(aris::core::Msg(std::to_string(i)));
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}

			client.stop();
			});
	}

	for (int i = 0; i < num_clients; ++i)
		th[i].join();


	std::cout << "finished" << std::endl;
}

int main(){
	//test2();
	//test3();
	//test4();
	test5();

	return 0;
}

