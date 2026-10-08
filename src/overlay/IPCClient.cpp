// SPDX-License-Identifier: AGPL-3.0-only

#include "IPCClient.h"
#include "OverlayLog.h"

#include <string>
#include <stdexcept>
#include <algorithm>
#include <cstring>

IPCClient::~IPCClient()
{
	if (pipe && pipe != INVALID_HANDLE_VALUE)
		CloseHandle(pipe);
}

void IPCClient::Disconnect(const std::string &error, double time)
{
	if (Connected() || lastError != error)
		overlaylog::Write("driver state=down error=", std::quoted(error));
	if (pipe != INVALID_HANDLE_VALUE)
		CloseHandle(pipe);
	pipe = INVALID_HANDLE_VALUE;
	lastError = error;
	nextAttempt = time + retryDelay;
	retryDelay = (std::min)(retryDelay * 2, 30.0);
	fprintf(stderr, "IPC: %s\n", error.c_str());
}

bool IPCClient::Connect(double time)
{
	if (Connected() || time < nextAttempt)
		return false;
	try
	{
		ConnectInternal();
		retryDelay = 1;
		lastError.clear();
		overlaylog::Write("driver state=up error=", std::quoted(lastError));
		return true;
	}
	catch (const std::exception &error)
	{
		Disconnect(error.what(), time);
		return false;
	}
}

protocol::Response IPCClient::SendBlocking(const protocol::Request &request)
{
	Send(request);
	return Receive();
}

void IPCClient::Send(const protocol::Request &request)
{
	DWORD bytesWritten;
	BOOL success = WriteFile(pipe, &request, sizeof request, &bytesWritten, 0);
	if (!success)
	{
		throw std::runtime_error("Error writing IPC request. Error: " + overlaylog::WindowsError(GetLastError()));
	}
}

protocol::Response IPCClient::Receive()
{
	protocol::Response response(protocol::ResponseInvalid);
	DWORD bytesRead;

	BOOL success = ReadFile(pipe, &response, sizeof response, &bytesRead, 0);
	if (!success)
	{
		DWORD lastError = GetLastError();
		if (lastError != ERROR_MORE_DATA)
		{
			throw std::runtime_error("Error reading IPC response. Error: " + overlaylog::WindowsError(lastError));
		}
	}

	if (bytesRead != sizeof response)
	{
		throw std::runtime_error("Invalid IPC response with size " + std::to_string(bytesRead));
	}

	return response;
}

void IPCClient::ConnectInternal()
{
	LPCTSTR pipeName = TEXT(OPENVR_SPACECALIBRATOR_PIPE_NAME);
	pipe = CreateFile(pipeName, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	if (pipe == INVALID_HANDLE_VALUE)
	{
		throw std::runtime_error("Driver is unavailable. " + overlaylog::WindowsError(GetLastError()));
	}

	DWORD mode = PIPE_READMODE_MESSAGE;
	if (!SetNamedPipeHandleState(pipe, &mode, 0, 0))
	{
		throw std::runtime_error("Couldn't set pipe mode. Error: " + overlaylog::WindowsError(GetLastError()));
	}

	protocol::Request request;
	std::memset(&request, 0, sizeof request);
	request.type = protocol::RequestHandshake;
	Send(request);
	auto response = Receive();

	if (response.type != protocol::ResponseHandshake || response.protocol.version != protocol::Version)
	{
		throw std::runtime_error(
			"Driver protocol differs. Client: " +
			std::to_string(protocol::Version) + ", Driver: " +
			std::to_string(response.protocol.version) + "."
		);
	}
}
