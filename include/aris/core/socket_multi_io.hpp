﻿#ifndef ARIS_CORE_SOCKET_MULTI_IO_H_
#define ARIS_CORE_SOCKET_MULTI_IO_H_

#include <aris_lib_export.h>
#include <aris/core/object.hpp>
#include <aris/core/msg.hpp>

namespace aris::core{
	class ARIS_API SocketMultiIo{
	public:
		enum class State{
			IDLE = 0,
			WAITING_FOR_CONNECTION,
			WORKING,
		};
		enum class Type{
			TCP,
			UDP,
			WEB,
			TCP_RAW,
			UDP_RAW,
			WEB_RAW
		};

	public:
		auto isConnected()->bool;
		auto state()->State;
		auto startServer(const std::string &port = std::string())->void;
		auto connect(const std::string &remote_ip = std::string(), const std::string &port = std::string())->void;
		auto stop()->void;
		auto sendMsg(const aris::core::MsgBase &data)->void;
		auto sendRawData(const char *data, int size)->void;

		auto port()const->const std::string &;
		auto setPort(const std::string &port)->void;
		auto remoteIP()const->const std::string &;
		auto setRemoteIP(const std::string &remote_ip)->void;
		auto connectType()const->Type;
		auto setConnectType(const Type type)->void;
		// connect timeout: milliseconds
		// 若想用阻塞模式，将该值设为-1
		// 默认值为 -1
		auto connectTimeoutMs()const->std::int64_t;
		auto setConnectTimeoutMs(std::int64_t time_out_ms)->void;

		auto setOnReceivedMsg(std::function<int(SocketMultiIo*, aris::core::Msg &)> = nullptr)->void;
		auto setOnReceivedRawData(std::function<int(SocketMultiIo*, const char *data, int size)> = nullptr)->void;
		auto setOnReceivedConnection(std::function<int(SocketMultiIo*, const char* remote_ip, int remote_port)> = nullptr)->void;
		auto setOnLoseConnection(std::function<int(SocketMultiIo*)> = nullptr)->void;

		virtual ~SocketMultiIo();
		SocketMultiIo(const std::string &name = "socket", const std::string& remote_ip = "", const std::string& port = "", Type type = Type::TCP);
		SocketMultiIo(const SocketMultiIo & other) = delete;
		SocketMultiIo(SocketMultiIo && other)noexcept;
		SocketMultiIo &operator=(const SocketMultiIo& other) = delete;
		SocketMultiIo &operator=(SocketMultiIo&& other)noexcept;

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};
}


#endif