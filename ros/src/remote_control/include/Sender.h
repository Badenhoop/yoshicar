#include <utility>

//
// Created by philipp on 21.04.19.
//

#ifndef REMOTE_CONTROL_SENDER_H
#define REMOTE_CONTROL_SENDER_H

#include <asionet/DatagramReceiver.h>
#include <asionet/DatagramSender.h>
#include <asionet/Timer.h>
#include <asionet/Monitor.h>
#include "Message.h"
#include <unordered_map>

namespace remoteControl
{

class Sender : public std::enable_shared_from_this<Sender>
{
public:
	Sender(
		asionet::Context & context,
		const std::string & receiverAddress,
		std::uint16_t receiverPort,
		std::uint16_t senderPort)
		: receiver(context, senderPort)
		, sender(context)
		, timer(context)
		, receiverEndpoint(boost::asio::ip::address::from_string(receiverAddress), receiverPort)
	{}

	void setControlValue(int controlType, double controlValue);

	void run();

private:
	static const asionet::time::Duration sendInterval;

	asionet::DatagramReceiver<ResponseMessage> receiver;
	asionet::DatagramSender<ControlMessage> sender;
	asionet::Timer timer;
	int id = 0;
	asionet::utils::Monitor<std::unordered_map<int, double>> controlValues;
	asionet::DatagramReceiver<ResponseMessage>::Endpoint receiverEndpoint;

	void send();

	void receive();
};

}

#endif //REMOTE_CONTROL_SENDER_H
