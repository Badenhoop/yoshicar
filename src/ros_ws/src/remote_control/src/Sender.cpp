//
// Created by philipp on 21.04.19.
//

#include <iostream>
#include "../include/Sender.h"

namespace remoteControl
{

const asionet::time::Duration Sender::sendInterval = std::chrono::milliseconds(50);

void Sender::run()
{
	send();
	receive();
}

void Sender::setControlValue(int controlType, double controlValue)
{
	controlValues([&](auto & values) { values[controlType] = controlValue; });
}

void Sender::send()
{
	auto self = shared_from_this();
	using namespace std::chrono_literals;
	timer.startPeriodicTimeout(sendInterval, [self]
	{
		self->controlValues(
			[&](auto & values)
			{
				for (const auto & pair : values)
				{
					auto controlType = pair.first;
					auto controlValue = pair.second;
					self->sender.asyncSend(ControlMessage{self->id, controlType, controlValue},
					                       self->receiverEndpoint, 50ms, [self](auto && ...) {});
				}
			});
	});
}

void Sender::receive()
{
	using namespace std::chrono_literals;
	auto self = shared_from_this();
	receiver.asyncReceive(
		asionet::time::Duration::max(),
		[self](const auto & error, const auto & responseMessage, const auto & senderEndpoint)
		{
			if (error)
			{
				self->receive();
				return;
			}

			self->id = responseMessage.id;

			self->receive();
		});
}

}