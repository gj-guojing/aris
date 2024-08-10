#include <cstring>
#include <fstream>
#include <ctime>
#include <mutex>
#include <algorithm>
#include <iostream>

#ifdef WIN32
#include <ws2tcpip.h>
#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif
#endif

#ifdef UNIX
#include <pthread.h>
#include <semaphore.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <signal.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <fcntl.h>
#endif

#include "aris/core/msg.hpp"

namespace aris::core
{
	auto MsgBase::copy(const std::string &str)->void { copy(str.data(), static_cast<aris::core::MsgSize>(str.size())); }
	auto MsgBase::copy(const void *src, MsgSize data_size)->void { resize(data_size); copyAt(src, data_size, 0); }
	auto MsgBase::copyAt(const void *src, MsgSize data_size, MsgSize at_this_pos_of_msg)->void
	{
		if ((data_size + at_this_pos_of_msg) > size())resize(data_size + at_this_pos_of_msg);
		std::copy_n(static_cast<const char *>(src), data_size, data() + at_this_pos_of_msg);
	}
	auto MsgBase::copyMore(const void *src, MsgSize data_size)->void { copyAt(src, data_size, size()); }
	auto MsgBase::paste(void *tar, MsgSize data_size) const->void { std::copy_n(data(), std::min(size(), data_size), static_cast<char*>(tar)); }
	auto MsgBase::paste(void *tar) const->void { std::copy_n(data(), size(), static_cast<char*>(tar)); }
	auto MsgBase::pasteAt(void *tar, MsgSize data_size, MsgSize at_this_pos_of_msg) const->void
	{
		std::copy_n(data() + at_this_pos_of_msg, std::min(data_size, size() - at_this_pos_of_msg), static_cast<char*>(tar));
	}

	auto MsgBase::setDestinationSockaddrin(void* sockaddr_in) -> void {
		std::copy_n((char*)sockaddr_in, 16, (char*)&header().msg_type_);
	}
	auto MsgBase::destinationSockaddrin()const -> void* {
		return (void*)&header().msg_type_;
	}
	auto MsgBase::destinationIpStr()const -> std::string {
		return std::string(inet_ntoa(reinterpret_cast<const struct sockaddr_in*>(destinationSockaddrin())->sin_addr));
	}
	auto MsgBase::destinationPort()const -> int {
		return ntohs(reinterpret_cast<const struct sockaddr_in*>(destinationSockaddrin())->sin_port);
	}
	auto MsgBase::setSourceSockaddrin(void* sockaddr_in) -> void {
		std::copy_n((char*)sockaddr_in, 16, (char*)&header().reserved2_);
	}
	auto MsgBase::sourceSockaddrin()const -> void* {
		return (void*)&header().reserved2_;
	}
	auto MsgBase::sourceIpStr()const -> std::string {
		return std::string(inet_ntoa(reinterpret_cast<const struct sockaddr_in*>(sourceSockaddrin())->sin_addr));
	}
	auto MsgBase::sourcePort()const -> int {
		return ntohs(reinterpret_cast<const struct sockaddr_in*>(sourceSockaddrin())->sin_port);
	}

	auto Msg::swap(Msg &other)->void { std::swap(data_, other.data_); std::swap(capacity_, other.capacity_); }
	auto Msg::resize(MsgSize data_size)->void
	{
		if (capacity_ < data_size)
		{
			capacity_ = std::max(data_size, capacity_ * 2);
			std::unique_ptr<char[]> other(new char[sizeof(MsgHeader) + capacity_]());
			std::copy_n(data_.get(), sizeof(MsgHeader) + size(), other.get());
			std::swap(data_, other);
		}

		header().msg_size_ = data_size;
	}
	auto Msg::header()->MsgHeader& { return *reinterpret_cast<MsgHeader*>(data_.get()); }
	auto Msg::header()const->const MsgHeader& { return *reinterpret_cast<const MsgHeader*>(data_.get()); }

	
	Msg::~Msg() = default;
	Msg::Msg(MsgID msg_id, MsgSize size) :data_(std::make_unique<char[]>(sizeof(MsgHeader) + size)), capacity_(size)
	{
		header().msg_id_ = 0;
		header().msg_size_ = size;
		header().msg_type_ = 0;
		header().reserved1_ = 0;
		header().reserved2_ = 0;
		header().reserved3_ = 0;
	}
	Msg::Msg(const std::string &str) :data_(std::make_unique<char[]>(sizeof(MsgHeader) + str.size())), capacity_(static_cast<MsgSize>(str.size()))
	{
		std::copy(str.begin(), str.end(), data());
		header().msg_id_ = 0;
		header().msg_size_ = static_cast<MsgSize>(str.size());
		header().msg_type_ = 0;
		header().reserved1_ = 0;
		header().reserved2_ = 0;
		header().reserved3_ = 0;
	}
	Msg::Msg(const MsgBase &other) : data_(std::make_unique<char[]>(sizeof(MsgHeader) + other.size())), capacity_(other.size())
	{
		std::copy_n(reinterpret_cast<const char*>(&other.header()), other.size() + sizeof(MsgHeader), reinterpret_cast<char*>(&header()));
	}
	Msg::Msg(const Msg& other) : data_(std::make_unique<char[]>(sizeof(MsgHeader) + other.size())), capacity_(other.size())
	{
		std::copy_n(other.data_.get(), sizeof(MsgHeader) + other.size(), data_.get());
	}
	Msg::Msg(Msg&& other)noexcept { swap(other); }
	Msg& Msg::operator=(Msg &&other)noexcept { swap(other); return (*this); }

	auto MsgStreamBuf::overflow(int_type c)->int_type{
		msg_->resize(msg_->capacity());// 保证在下次resize的时候，所有数据都会被copy，这是因为在resize重新分配内存时，不是按照capacity来copy
		msg_->resize(msg_->capacity() + 1);
		setp(msg_->data() + msg_->size(), msg_->data() + msg_->capacity());
		*(pptr() - 1) = c;
		return c;
	}
	auto MsgStreamBuf::sync()->int{
		msg_->resize(msg_->capacity() - static_cast<MsgSize>(epptr() - pptr()));
		return 0;
	}
	auto MsgStreamBuf::reset()->void { setp(msg_->data(), msg_->data() + msg_->capacity()); }
	MsgStreamBuf::MsgStreamBuf(MsgBase& msg) :msg_(&msg), std::streambuf() {
		setp(msg.data(), msg.data() + msg.capacity());
	};
}
