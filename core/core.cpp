//
// Created by urbschaj on 2019-09-30.
//

#include <thread>
#include <fmt/format.h>
#include <boost/asio.hpp>
#include "core.h"

namespace TMCM {

	Exception::Exception(std::string error)
	: error(error)
			{}
	const char* Exception::what() const noexcept{
		return error.c_str();
	}

	///////////////////////////////////////////////////////////////

	uint8_t BuildChecksum(const std::array<uint8_t, 8>& data) {
		uint8_t checksum = 0;
		for(uint8_t item : data) {
			checksum += item;
		}
		return checksum;
	}

	///////////////////////////////////////////////////////////////

	TMCMCommand::TMCMCommand(uint8_t module, CommandCodes command, uint8_t type, uint8_t motor, std::array<uint8_t, 4> payload)
			: module(module), command(command), type(type), motor(motor), payload(payload)
	{
		std::array<uint8_t, 8> buffer = {
				module,
				(uint8_t)command,
				type,
				motor,
				payload[0],
				payload[1],
				payload[2],
				payload[3]
		};
		uint8_t checksum = BuildChecksum(buffer);
		std::copy(buffer.begin(), buffer.end(), commandBuffer.begin());
		commandBuffer[8] = checksum;
	}
	const CommandResponseBuffer& TMCMCommand::GetData() const {
		return commandBuffer;
	}

	std::string TMCMCommand::to_string() const {
		auto value = ParsePayload<int32_t>(payload);
		return fmt::format("{{ module: {}, command: {} type: {} motor: {} value: {} (as int), chk: {} }}", module, command, type, motor, value, commandBuffer[8]);
	}

	TMCMResoponse::TMCMResoponse(uint8_t replyAddress, uint8_t moduleAddress, uint8_t status, uint8_t command, std::array<uint8_t, 4> payload, CommandResponseBuffer raw)
			: replyAddress(replyAddress),
			  moduleAddress(moduleAddress),
			  status(status),
			  command(command),
			  payload(payload),
			  raw(raw)
	{}

	optional<TMCMResoponse> TMCMResoponse::ParseResponse(const CommandResponseBuffer& response) {
		auto data = SliceBuffer<0, 8>(response);
		if(BuildChecksum(data) != response[8]) {
			return optional<TMCMResoponse>{};
		}
		auto payload = SliceBuffer<4, 4>(response);
		return TMCMResoponse {
				response[0],
				response[1],
				response[2],
				response[3],
				payload,
				response
		};
	}

	uint8_t TMCMResoponse::GetReplyAddress() const {
		return replyAddress;
	}

	uint8_t TMCMResoponse::GetModuleAddress() const {
		return moduleAddress;
	}

	ReturnCodes TMCMResoponse::GetStatus() const {
		return static_cast<ReturnCodes>(status);
	}

	uint8_t TMCMResoponse::GetCmmand() const {
		return command;
	}

	int32_t TMCMResoponse::PayloadAsInt() {
		int32_t out = ParsePayload<int32_t>(payload);
		return out;
	}

	uint32_t TMCMResoponse::PayloadAsUInt() {
		uint32_t out = ParsePayload<uint32_t>(payload);
		return out;
	}

	std::array<uint8_t, 4> TMCMResoponse::GetPayload() const {
		return payload;
	}

	CommandResponseBuffer TMCMResoponse::GetRaw() const {
		return raw;
	}

	std::string TMCMResoponse::to_string() const {
		auto value = ParsePayload<int32_t>(payload);
		return fmt::format("{{ replyAddress: {}, moduleAddress: {} status: {} command: {} value: {} (as int), chk: {} }}", replyAddress, moduleAddress, status, command, value, raw[8]);
	}

	////////////////////////////////////////////////////////////

	SerialInterface::SerialInterface(std::string serialDevice, int32_t baudRate, float timeout)
			: io(),
			  port(io),
			  timer(io),
			  timeout(boost::posix_time::milliseconds(int(timeout * 1000)))
	{
		using namespace boost::asio;
		auto opt_baudrate = serial_port_base::baud_rate(baudRate);
		auto opt_parity = serial_port_base::parity(serial_port_base::parity::none);
		auto opt_csize = serial_port_base::character_size(8);
		auto opt_flow = serial_port_base::flow_control(serial_port_base::flow_control::none);
		auto opt_stop = serial_port_base::stop_bits(serial_port_base::stop_bits::one);
		try {
			port.open(serialDevice);
			port.set_option(opt_baudrate);
			port.set_option(opt_parity);
			port.set_option(opt_csize);
			port.set_option(opt_flow);
			port.set_option(opt_stop);
		} catch(const std::exception& ex) {
			throw;
		}
	}

	SerialInterface::~SerialInterface() {
		if(port.is_open()) {
			port.close();
		}
	}

	TMCMResoponse SerialInterface::writeRead(const TMCMCommand& command) {
		optional<CommandResponseBuffer> response;
		try {
			response = writeRead(command.GetData());
		} catch(const std::exception& ex) {
			auto err = ex.what();
			throw Exception(fmt::format("unable so send/recv, error: {} (cmd: {})", command.to_string(), err));
		}
		if(!response) {
			throw Exception(fmt::format("no response data received for cmd: {}", command.to_string()));
		}
		auto unpacked = TMCM::TMCMResoponse::ParseResponse(*response);
		if(unpacked) {
			return *unpacked;
		}
		throw Exception("response data invalid (checksum mismatch), unable to unpack");
	}

	void SerialInterface::write(const TMCMCommand& command) {
		try {
			write(command.GetData());
		} catch(const std::exception& ex) {
			auto err = ex.what();
			throw Exception(fmt::format("unable so send, error: {} (cmd: {})", command.to_string(), err));
		}
	}

	void SerialInterface::write(const CommandResponseBuffer& command) {
		using namespace boost;
		asio::write(port, asio::buffer(command.begin(), command.size()));
	}

	optional<CommandResponseBuffer> SerialInterface::writeRead(const CommandResponseBuffer& command)
	{
		using namespace boost;
		enum class FinishReason {
			FAILURE = 0,
			SUCCESS = 1
		};

		CommandResponseBuffer outBuffer;
		FinishReason finishReason = FinishReason::FAILURE;

		asio::write(port, asio::buffer(command.begin(), command.size()));
		asio::async_read(port, asio::buffer(outBuffer.begin(), outBuffer.size()),
		                 [&, this](const boost::system::error_code& ec, std::size_t bytes_transferred){
			                 timer.cancel();
			                 if (bytes_transferred == outBuffer.size() && ec == boost::system::errc::success) {
				                 finishReason = FinishReason::SUCCESS;
			                 }
		                 }
		);

		timer.expires_from_now(posix_time::seconds(1));
		timer.async_wait([&, this](const boost::system::error_code& error){
			// if we stopped the timer in the success handler we wil get an aborted
			// event on the next run, we simply ignore that an only react to the
			// success event
			if(error == boost::system::errc::success) {
				port.cancel();
			}
		});

		io.run();
		// must be called to clear the stopped state, otherwise run() will immediately return
		// on the next invocation
		io.reset();

		if(finishReason == FinishReason::SUCCESS) {
			return outBuffer;
		}
		return optional<CommandResponseBuffer>();
	}

	//void SerialInterface::write(const std::string& s) {}
	//std::string SerialInterface::read() {}

	///////////////////////////////////////////////////

	namespace Builder {
		TMCMCommand RotateRight(Module module, Motor motor, uint32_t velocity) {
			return TMCMCommand(module, CommandCodes::RotateRight, CommandTypes::TYPE_UNUSED, motor, velocity);
		}
		TMCMCommand RotateLeft(Module module, Motor motor, uint32_t velocity) {
			return TMCMCommand(module, CommandCodes::RotateLeft, CommandTypes::TYPE_UNUSED, motor, velocity);
		}
		TMCMCommand StopMotor(Module module, Motor motor) {
			return TMCMCommand(module, CommandCodes::StopMotor, CommandTypes::TYPE_UNUSED, motor, CommandTypes::VALUE_UNUSED);
		}
		TMCMCommand SetGlobalParameter(Module module, TypeParams::GlobalParamaters::Type type, TypeParams::GlobalParamaters::Value value, Bank bank) {
			return TMCMCommand(module, CommandCodes::SetGlobalParameter, static_cast<CommandTypes::TYPE_TYPE>(type), bank, static_cast<CommandTypes::VALUE_TYPE>(value));
		}
		TMCMCommand GetGlobalParameter(Module module, TypeParams::GlobalParamaters::Type type, Bank bank) {
			return TMCMCommand(module, CommandCodes::GetGlobalParameter, static_cast<CommandTypes::TYPE_TYPE>(type), bank, CommandTypes::VALUE_UNUSED);
		}
		TMCMCommand SetAxisParemater(Module module, TypeParams::AxisParamaters::Type type, Motor motor, int32_t data) {
			return TMCMCommand(module, CommandCodes::SetAxisParameter, static_cast<CommandTypes::TYPE_TYPE>(type), motor, data);
		}
		TMCMCommand GetAxisParamater(Module module, TypeParams::AxisParamaters::Type type, Motor motor) {
			return TMCMCommand(module, CommandCodes::GetAxisParameter, static_cast<CommandTypes::TYPE_TYPE>(type), motor, CommandTypes::VALUE_UNUSED);
		}
		TMCMCommand MoveToPosition(Module module, Motor motor, int32_t pos) {
			return TMCMCommand(module, CommandCodes::MoveToPosition, static_cast<CommandTypes::TYPE_TYPE>(TypeParams::MoveToPosition::Type::Absolute), motor, pos);
		}
		TMCMCommand Restart(Module module) {
			return TMCMCommand(module, CommandCodes::SoftReset, CommandTypes::TYPE_UNUSED, CommandTypes::BANK_UNUSED, CommandTypes::VALUE_UNUSED);
		}
		TMCMCommand GetFirmwareVersion(Module module) {
			return TMCMCommand(module, CommandCodes::GetFirmwareVersion, static_cast<CommandTypes::TYPE_TYPE>(TypeParams::FirmwareVersion::Type::BinaryFormat), CommandTypes::BANK_UNUSED, CommandTypes::VALUE_UNUSED);
		}
		TMCMCommand ReferenceSearch(Module module, Motor motor, TypeParams::ReferenceSearch::Type type) {
			return TMCMCommand(module, CommandCodes::ReferenceSearch, static_cast<CommandTypes::TYPE_TYPE>(type), motor, CommandTypes::VALUE_UNUSED);
		}
	}

	///////////////////////////////////////////////////

	ControllerInterface::ControllerInterface(std::string serialDevice, int32_t baudRate, const std::vector<int32_t>& modules)
	{
		SetupConnection(serialDevice, baudRate, modules);
	}

	ControllerInterface::~ControllerInterface() {}

	TMCMResoponse ControllerInterface::writeRead(const TMCMCommand& command) {
		std::lock_guard<std::mutex> lock(mtx);
		if(!si) {
			throw Exception("no serial interface created");
		}
		return si->writeRead(command);
	}

	void ControllerInterface::SetupConnection(std::string serialDevice, int32_t requestedRate, const std::vector<int32_t>& modules) {
		std::array<int32_t, 9> suppotedRates = {9600, 14400, 19200, 28800, 38400, 57600, 76800, 115200, 230400 };
		auto it = std::find(suppotedRates.begin(), suppotedRates.end(), requestedRate);
		if(it == suppotedRates.end()) {
			throw Exception(fmt::format("unsupported baud rate: {}", requestedRate));
		}
		try {
			si = std::make_unique<SerialInterface>(serialDevice, requestedRate, 1);
		} catch (const std::exception& ex) {
			throw Exception(fmt::format("unable to open connection. check serial device name '{}:{}' and cables. error: {}\n", serialDevice, requestedRate, ex.what()));
		}
		for(auto module : modules) {
			try {
				// if we receive a valid answer we found our baud rate
				auto firmwareCmd = Builder::GetFirmwareVersion(Module(module));
				auto response = si->writeRead(firmwareCmd);
				fmt::print("module {} found, firmware: {}\n", module, response.PayloadAsInt());
				//activeModules[module] = response.PayloadAsInt();
			} catch(const std::exception& ex) {
				throw Exception(fmt::format("module {} not found (check module baud rate, trying to connect with {})\n", module, requestedRate));
			}
		}
	}

	//////////////////////////////////////////////////////


	Core::Core()
		: tmcmInterface(nullptr)
	{}

	Core::~Core() {}

	void Core::Init(std::string serialDevice, int32_t baudRate, const std::vector<int32_t>& modules) {
		tmcmInterface = std::make_unique<ControllerInterface>(serialDevice, baudRate, modules);
	}

	std::unique_ptr<ControllerInterface>& Core::GetInterface() {
		return tmcmInterface;
	}

	//////////////////////////////////////////////////////

	Core& GetCore() {
		static Core core;
		return core;
	}


}
