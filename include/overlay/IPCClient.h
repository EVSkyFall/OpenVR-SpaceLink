// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <Windows.h>
#include <string>
#include <optional>

#include "Protocol.h"

class IPCClient
{
public:
	~IPCClient();

	bool Connect(double time = 0);
	void Disconnect(const std::string &error, double time);
	bool Connected() const { return pipe != INVALID_HANDLE_VALUE; }
	const std::string &LastError() const { return lastError; }
	protocol::Response SendBlocking(const protocol::Request &request);
	std::optional<protocol::DriftState> GetDriftState();

	void Send(const protocol::Request &request);
	protocol::Response Receive();

private:
	void ConnectInternal();

	HANDLE pipe = INVALID_HANDLE_VALUE;
	std::string lastError;
	double nextAttempt = 0, retryDelay = 1;
};