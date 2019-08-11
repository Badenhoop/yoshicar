//
// Created by philipp on 21.04.19.
//

#include <iostream>
#include "../include/Receiver.h"

namespace remoteControl
{

void Receiver::run()
{
	startTimeout();
	receive();
}

void Receiver::startTimeout()
{
	auto self = shared_from_this();
	timer.startTimeout(connectionTimeout, [self]
	{
		self->id += 1;
		self->connectionTimeoutCallback();
	});
}

void Receiver::receive()
{
	auto self = shared_from_this();
	using namespace std::chrono_literals;
	receiver.asyncReceive(
		asionet::time::Duration::max(),
		[self](const auto & error, const auto & controlMessage, const auto & senderEndpoint)
		{
			if (error)
			{
				self->receive();
				return;
			}

			if (controlMessage.id == self->id)
			{
				self->controlCallback(controlMessage.controlType, controlMessage.controlValue);
				self->startTimeout();
			}

			using namespace std::chrono_literals;
			decltype(senderEndpoint) destEndpoint{senderEndpoint.address(), self->senderPort};
			self->sender.asyncSend(ResponseMessage{self->id}, destEndpoint, 100ms, [self](auto && ...){});

			self->receive();
		});
}

}