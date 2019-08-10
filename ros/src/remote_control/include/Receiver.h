//
// Created by philipp on 20.04.19.
//

#ifndef REMOTE_CONTROL_RECEIVER_H
#define REMOTE_CONTROL_RECEIVER_H

#include <asionet/DatagramReceiver.h>
#include <asionet/DatagramSender.h>
#include <asionet/Timer.h>
#include "Common.h"
#include "Message.h"

namespace remoteControl
{

class Receiver : public std::enable_shared_from_this<Receiver>
{
public:
	using ConnectionTimeoutCallback = std::function<void()>;
	using ControlCallback = std::function<void(int controlType, double controlValue)>;

	Receiver(
		asionet::Context & context,
		asionet::time::Duration connectionTimeout,
		ConnectionTimeoutCallback connectionTimeoutCallback,
		ControlCallback controlCallback)
		: context(context)
		, receiver(context, RECEIVER_PORT)
		, sender(context)
		, timer(context)
		, connectionTimeout(connectionTimeout)
		, connectionTimeoutCallback(connectionTimeoutCallback)
		, controlCallback(controlCallback)
	{}

	void run();

private:
	asionet::Context & context;
	asionet::DatagramReceiver<ControlMessage> receiver;
	asionet::DatagramSender<ResponseMessage> sender;
	asionet::Timer timer;
	asionet::time::Duration connectionTimeout;
	int id = 0;
	ConnectionTimeoutCallback connectionTimeoutCallback;
	ControlCallback controlCallback;

	void startTimeout();

	void receive();
};

}

#endif //REMOTE_CONTROL_RECEIVER_H
