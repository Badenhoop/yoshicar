//
// Created by philipp on 20.04.19.
//

#ifndef REMOTE_CONTROL_MESSAGE_H
#define REMOTE_CONTROL_MESSAGE_H

#include <asionet/Message.h>
#include "json.hpp"

namespace remoteControl
{

struct ControlMessage
{
	int id;
	int controlType;
	double controlValue;
};

struct ResponseMessage
{
	int id;
};

}

namespace asionet
{
namespace message
{

template<>
struct Encoder<remoteControl::ControlMessage>
{
	void operator()(const remoteControl::ControlMessage & message, std::string & data) const
	{
		auto j = nlohmann::json{ {"id", message.id},
		                         {"controlType", message.controlType},
		                         {"controlValue", message.controlValue} };
		data = j.dump();
	}
};

template<>
struct Decoder<remoteControl::ControlMessage>
{
	template<typename ConstBuffer>
	void operator()(const ConstBuffer & buffer, remoteControl::ControlMessage & message) const
	{
		std::string s{buffer.begin(), buffer.end()};
		auto j = nlohmann::json::parse(s);
		message = remoteControl::ControlMessage{
			j.at("id").get<int>(),
			j.at("controlType").get<int>(),
			j.at("controlValue").get<double>()
		};
	}
};

template<>
struct Encoder<remoteControl::ResponseMessage>
{
	void operator()(const remoteControl::ResponseMessage & message, std::string & data) const
	{
		auto j = nlohmann::json{ {"id", message.id} };
		data = j.dump();
	}
};

template<>
struct Decoder<remoteControl::ResponseMessage>
{
	template<typename ConstBuffer>
	void operator()(const ConstBuffer & buffer, remoteControl::ResponseMessage & message) const
	{
		std::string s{buffer.begin(), buffer.end()};
		auto j = nlohmann::json::parse(s);
		message = remoteControl::ResponseMessage{
			j.at("id").get<int>()
		};
	}
};

}
}

#endif //REMOTE_CONTROL_MESSAGE_H
