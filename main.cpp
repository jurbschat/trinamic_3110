#include <iostream>
#include <fstream>
#include <thread>
#include <boost/asio.hpp>
#include <boost/container/static_vector.hpp>
#include <string>
#include <cstdint>
#include <type_traits>
#include <experimental/optional>
#include <unordered_map>
#include <fmt/format.h>
#include "stopwatch.h"

template<typename T>
using optional = std::experimental::optional<T>;

namespace tmcm_3110 {

	using CommandResponseBuffer = std::array<uint8_t, 9>;

	class Exception : public std::exception {
	public:
		Exception(std::string error)
				: error(error)
		{}
		const char* what() const noexcept{
			return error.c_str();
		}
	private:
		std::string error;
	};

	template<typename T>
	using IsFittingIntegral = typename std::enable_if<std::is_integral<T>::value && sizeof(T) == 4>::type;

	uint8_t BuildChecksum(const std::array<uint8_t, 8>& data) {
		uint8_t checksum = 0;
		for(uint8_t item : data) {
			checksum += item;
		}
		return checksum;
	}

	//TODO: does this work on arm?
	template<typename T, typename = IsFittingIntegral<T>>
	std::array<uint8_t, 4> BuildPayload(T value) {
		uint8_t byte1 =  value & 0x000000ff;
		uint8_t byte2 = (value & 0x0000ff00) >> 8;
		uint8_t byte3 = (value & 0x00ff0000) >> 16;
		uint8_t byte4 = (value & 0xff000000) >> 24;
		std::array<uint8_t, 4> out {byte4, byte3, byte2, byte1};
		return out;
	};

	//TODO: does this work on arm?
	template<typename T, typename = IsFittingIntegral<T>>
	T ParsePayload(const std::array<uint8_t, 4>& payload) {
		T out;
		out = (payload[0] << 24) | (payload[1] << 16) | (payload[2] << 8) | payload[3];
		return out;
	};

	template<size_t start, size_t count, typename T>
	std::array<typename T::value_type, count> SliceBuffer(const T& buffer) {
		static_assert(std::tuple_size<T>::value >= start + count, "invalid sizes for slice");
		std::array<typename T::value_type, count> out;
		std::copy(buffer.begin() + start, buffer.begin() + start + count, out.begin());
		return out;
	}

	enum class ReturnCodes : uint8_t {
		SUCCESS=100,
		COMMAND_LOADED=101,
		WRONG_CHECKSUM=1,
		INVALID_COMMAND=2,
		WRONG_TYPE=3,
		INVALID_VALUE=4,
		EEPROM_LOCKED=5,
		COMMAND_UNAVIABLE=6
	};

	enum class CommandCodes : uint8_t {
		RotateRight=1,              //  (ROR) Rotate right (m)
		RotateLeft=2,               //  (ROL) Rotate left (m)
		StopMotor=3,                //  (MST) motor stop (m)
		MoveToPosition=4,           //  (MVP) move to position (m)
		SetAxisParemater=5,         //  (SAP) set axis paramater (p)
		GetAxisParamater=6,         //  (GAP) get axis paramater (p)
		StoreAxisParamater=7,       //  (STAP) store axis paramater (p)
		RestoreAxisParamater=8,     //  (RSAP) restore axis paramater (p)
		SetGlobalParameter=9,       //  (SGP) set global paramater (p)
		GetGlobalParameter=10,      //  (GGP) get global paramater (p)
		StoreGlobalParamater=11,    //  (STGP) store global paramater (p)
		RestoreGlobalParamater=12,  //  (RSGP) restore global paramater (p)
		ReferenceSearch=13,         //  (RFS) reference search (?)
		SetOutput=14,               //  (SIO) set output (io)
		GetInput=15,                //  (GIO) get input (io)
		CALC=19,                    //  (CALC) calculate using the accu and const value (c)
		COMP=20,                    //  (COMP) compare accumulator with constant value (b)
		JC=21,                      //  (JC) jump connditional (b)
		JA=22,                      //  (JA) jump always (b)
		CSUB=23,                    //  (CSUB) call subroutine (b)
		RSUB=24,                    //  (RSUB) return from subroutine (b)
		EnableInterrupt=25,         //  (EI) enable interrupt (int)
		DisableInterrupt=26,        //  (DI) disable interrupt (int)
		WAIT=27,                    //  (WAIT) wait for event (b)
		STOP=28,                    //  (STOP) end of tmcl program (b)
		StoreCoordinate=30,         //  (SCO) store coordinate (m)
		GetCordinate=31,            //  (GCO) get coordinate (m)
		CaptureCoordinate=32,       //  (CCO) capture coordinate (m)
		CALCX=33,                   //  (CALCX) calculate using the accu and x register (c)
		AAP=34,                     //  (AAP) copy accu to axis param (c)
		AGP=35,                     //  (AGP) copy accu to global param (c)
		ClearErrorFlag=36,          //  (CLE) clear error flag (?)
		VECT=37,                    //  (VECT) set interrupt vector (int)
		RETI=38,                    //  (RETI) return from interrupt (int)
		ACO=39,                     //  (ACO) copy accu coordinate  (c)
		GetFirmwareVersion=136,     // ... :D
		SoftReset=255               //  cpu power cycle
	};

	namespace CommandTypes {
		using TYPE_TYPE = int8_t;
		using VALUE_TYPE = int32_t;

		const TYPE_TYPE TYPE_UNUSED = 0;
		const TYPE_TYPE MOTOR_UNUSED = 0;
		const TYPE_TYPE BANK_UNUSED = 0;
		const VALUE_TYPE VALUE_UNUSED = 0;
	}

	namespace TypeParams
	{
		namespace FirmwareVersion {
			enum class Type {
				StringFormat=0,
				BinaryFormat=1
			};
		}
		namespace GlobalParamaters {
			enum class Type {
				BaudRate=65,
				SerialAddress=66,
				AsciiMode=67,
				SerialHeartbeat=68,
				// ...
				TelegramPauseTime=75,
				SerialHostAddress=76,
				EndSwitchPolarity=79
			};
			enum class Value {
				Rate_9600=0,
				Rate_14400=1,
				Rate_19200=2,
				Rate_28800=3,
				Rate_38400=4,
				Rate_57600=5,
				Rate_76800=6,
				Rate_115200=7,
				Rate_230400=8,
			};
		}
	}

	class TMCMCommand
	{
	public:
		template<typename T, typename = IsFittingIntegral<T>>
		TMCMCommand(uint8_t module, CommandCodes command, uint8_t type, uint8_t motor, T payload)
			: TMCMCommand(module, command, type, motor, BuildPayload(payload))
		{}

		TMCMCommand(uint8_t module, CommandCodes command, uint8_t type, uint8_t motor, std::array<uint8_t, 4> payload)
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
		const CommandResponseBuffer& GetData() const {
			return commandBuffer;
		}

		std::string to_string() const {
			auto value = ParsePayload<int32_t>(payload);
			return fmt::format("{{ module: {}, command: {} type: {} motor: {} value: {} (as int), chk: {} }}", module, command, type, motor, value, commandBuffer[8]);
		}
	private:
		CommandResponseBuffer commandBuffer;
		uint8_t module;
		CommandCodes command;
		uint8_t type;
		uint8_t motor;
		std::array<uint8_t, 4> payload;
	};

	class TMCMResoponse
	{
		uint8_t replyAddress;
		uint8_t moduleAddress;
		uint8_t status;
		uint8_t command;
		std::array<uint8_t, 4> payload;
		CommandResponseBuffer raw;

		TMCMResoponse(uint8_t replyAddress, uint8_t moduleAddress, uint8_t status, uint8_t command, std::array<uint8_t, 4> payload, CommandResponseBuffer raw)
			: replyAddress(replyAddress),
			moduleAddress(moduleAddress),
			status(status),
			command(command),
			payload(payload),
			raw(raw)
		{}
	public:
		static optional<TMCMResoponse> ParseResponse(const CommandResponseBuffer& response) {
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
		uint8_t GetReplyAddress() const {
			return replyAddress;
		}
		uint8_t GetModuleAddress() const {
			return moduleAddress;
		}
		uint8_t GetStatus() const {
			return status;
		}
		uint8_t GetCmmand() const {
			return command;
		}
		int32_t PayloadAsInt() {
			int32_t out = ParsePayload<int32_t>(payload);
			return out;
		}
		uint32_t PayloadAsUInt() {
			uint32_t out = ParsePayload<uint32_t>(payload);
			return out;
		}
		std::array<uint8_t, 4> GetPayload() const {
			return payload;
		}
		CommandResponseBuffer GetRaw() const {
			return raw;
		}
		std::string to_string() const {
			auto value = ParsePayload<int32_t>(payload);
			return fmt::format("{{ replyAddress: {}, moduleAddress: {} status: {} command: {} value: {} (as int), chk: {} }}", replyAddress, moduleAddress, status, command, value, raw[8]);
		}
	};

	class SerialInterface
	{
	public:
		SerialInterface(std::string serialDevice, int32_t baudRate, float timeout)
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

		~SerialInterface() {
			if(port.is_open()) {
				port.close();
			}
		}

		TMCMResoponse writeRead(const TMCMCommand& command) {
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
			auto unpacked = tmcm_3110::TMCMResoponse::ParseResponse(*response);
			if(unpacked) {
				return *unpacked;
			}
			throw Exception("response data invalid, unable to unpack");
		}

		void write(const TMCMCommand& command) {
			try {
				write(command.GetData());
			} catch(const std::exception& ex) {
				auto err = ex.what();
				throw Exception(fmt::format("unable so send, error: {} (cmd: {})", command.to_string(), err));
			}
		}

		void write(const CommandResponseBuffer& command) {
			using namespace boost;
			asio::write(port, asio::buffer(command.begin(), command.size()));
		}

		optional<CommandResponseBuffer> writeRead(const CommandResponseBuffer& command)
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

		void write(const std::string& s) {}
		std::string read() {}

	private:
		boost::asio::io_service io;
		boost::asio::serial_port port;
		boost::asio::deadline_timer timer;
		boost::posix_time::time_duration timeout;
	};

	class Module {
		uint8_t module;
	public:
		explicit Module(uint8_t module)
			: module(module)
		{}
		operator uint8_t() const {
			return module;
		}
	};

	class Motor {
		uint8_t module;
	public:
		explicit Motor(uint8_t module)
			: module(module)
		{}
		operator uint8_t() const {
			return module;
		}
	};

	using Bank = Motor;

	namespace cb {
		TMCMCommand RotateRight(Module module, Motor motor, uint32_t velocity) {
			return TMCMCommand(module, CommandCodes::RotateRight, CommandTypes::TYPE_UNUSED, motor, velocity);
		}
		TMCMCommand RotateLeft(Module module, Motor motor, uint32_t velocity) {
			return TMCMCommand(module, CommandCodes::RotateLeft, CommandTypes::TYPE_UNUSED, motor, velocity);
		}
		TMCMCommand StopMotor(Module module, Motor motor) {
			return TMCMCommand(module, CommandCodes::StopMotor, CommandTypes::TYPE_UNUSED, motor, CommandTypes::VALUE_UNUSED);
		}
		TMCMCommand SetGlobalParameter(Module module, TypeParams::GlobalParamaters::Type type, TypeParams::GlobalParamaters::Value value, Bank bank = Bank(0)) {
			return TMCMCommand(module, CommandCodes::SetGlobalParameter, static_cast<CommandTypes::TYPE_TYPE>(type), bank, static_cast<CommandTypes::VALUE_TYPE>(value));
		}
		TMCMCommand GetGlobalParameter(Module module, TypeParams::GlobalParamaters::Type type, Bank bank = Bank(0)) {
			return TMCMCommand(module, CommandCodes::GetGlobalParameter, static_cast<CommandTypes::TYPE_TYPE>(type), bank, CommandTypes::VALUE_UNUSED);
		}
		/*BuildCommandMoveToPosition(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandSetAxisParemater(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandGetAxisParamater(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandStoreAxisParamater(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandRestoreAxisParamater(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandSetGlobalParamater(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandGetGlobalParameter(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandStoreGlobalParamater(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandRestoreGlobalParamater(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandReferenceSearch(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandSetOutput(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandGetInput(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandCALC(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandCOMP(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandJC(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandJA(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandCSUB(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandRSUB(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandEnableInterrupt(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandDisableInterrupt(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandWAIT(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandSTOP(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandStoreCoordinate(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandGetCordinate(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandCaptureCoordinate(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandCALCX(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandAAP(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandAGP(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandClearErrorFlag(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandVECT(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandRETI(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		BuildCommandACO(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}*/
		TMCMCommand Restart(Module module) {
			return TMCMCommand(module, CommandCodes::SoftReset, CommandTypes::TYPE_UNUSED, CommandTypes::BANK_UNUSED, CommandTypes::VALUE_UNUSED);
		}
		TMCMCommand GetFirmwareVersion(Module module) {
			return TMCMCommand(module, CommandCodes::GetFirmwareVersion, static_cast<CommandTypes::TYPE_TYPE>(TypeParams::FirmwareVersion::Type::BinaryFormat), CommandTypes::BANK_UNUSED, CommandTypes::VALUE_UNUSED);
		}
	}

	class TMCMInterface
	{
	public:
		TMCMInterface(std::string serialDevice, int32_t baudRate, const std::vector<int32_t>& modules)
		{
			SetupConnection(serialDevice, baudRate, modules);
		}

		~TMCMInterface() {}

	private:
		void SetupConnection(std::string serialDevice, int32_t requestedRate, const std::vector<int32_t>& modules) {
			std::array<int32_t, 9> suppotedRates = {9600, 14400, 19200, 28800, 38400, 57600, 76800, 115200, 230400 };
			auto it = std::find(suppotedRates.begin(), suppotedRates.end(), requestedRate);
			if(it == suppotedRates.end()) {
				fmt::print("unsupported baud rate: {}", requestedRate);
				return;
			}
			try {
				interface = std::make_unique<SerialInterface>(serialDevice, requestedRate, 1);
			} catch (const std::exception& ex) {
				fmt::print("unable to open connection. check serial device name '{}:{}' and cables. error: {}\n", serialDevice, requestedRate, ex.what());
				return;
			}
			for(auto module : modules) {
				try {
					// if we receive a valid answer we found our baud rate
					auto firmwareCmd = cb::GetFirmwareVersion(Module(module));
					auto response = interface->writeRead(firmwareCmd);
					fmt::print("module {} found, firmware: {}\n", module, response.PayloadAsInt());
					activeModules[module] = response.PayloadAsInt();
				} catch(const std::exception& ex) {
					fmt::print("module {} not found (check module baud rate, trying to connect with {})\n", module, requestedRate);
				}
			}
		}

		std::unique_ptr<SerialInterface> interface;
		std::unordered_map<int32_t, int32_t> activeModules;
	};

}

int main()
{
	using namespace tmcm_3110;

	TMCMInterface tmcm("/dev/ttyUSB0", 115200, {1, 2});

	/*using namespace tmcm_3110;
	SerialInterface interface("/dev/ttyUSB0", 115200, 1);
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	try {
		{
			auto cmd = cb::GetGlobalParameter(Module(1), TypeParams::GlobalParamaters::Type::BaudRate);
			auto response = interface.writeRead(cmd);
			auto val = response.PayloadAsInt();
			std::cout << val << std::endl;
		}
		std::this_thread::sleep_for(std::chrono::seconds(2));
		{
			tmcm_3110::TMCMCommand cmdMove(1, tmcm_3110::CommandCodes::RotateRight, 0, 0, 100);
			auto response = interface.writeRead(cmdMove);
			auto val = response.PayloadAsInt();
			std::cout << val << std::endl;
		}
		std::this_thread::sleep_for(std::chrono::seconds(2));
		util::StopWatch<> sw(true);
		for(int i = 0; i < 100; i++) {
			tmcm_3110::TMCMCommand cmdInfo(1, tmcm_3110::CommandCodes::GetAxisParamater, 140, 0, 0);
			auto response = interface.writeRead(cmdInfo);
			auto val = response.PayloadAsInt();
			std::cout << val << std::endl;
		}
		std::cout << 100 / sw.Seconds() << "cmd\\s" << std::endl;
		std::this_thread::sleep_for(std::chrono::seconds(2));
		{
			tmcm_3110::TMCMCommand cmdStop(1, tmcm_3110::CommandCodes::StopMotor, 0, 0, 0);
			auto response = interface.writeRead(cmdStop);
			auto val = response.PayloadAsInt();
			std::cout << val << std::endl;
		}
	} catch(const std::exception& ex) {
		fmt::print("error executing command: {}\n", ex.what());
	}*/

	return 0;
}


/*unsigned char move_cmd_right[] = {0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x03, 0xe8, 0xee};
unsigned char move_cmd_left[] = {0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x03, 0xe8, 0xee};
unsigned char stop_cmd[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04};
{
	auto cmd = std::string((char*)&move_cmd_left[0], std::size(move_cmd_left));
	serial.writeString(cmd);
	std::string response = serial.read(9);
	std::cout << response.size() << std::endl;
}
std::this_thread::sleep_for(std::chrono::seconds(2));
{
	auto cmd = std::string((char*)&stop_cmd[0], std::size(stop_cmd));
	serial.writeString(cmd);
	std::string response = serial.read(9);
	std::cout << response.size() << std::endl;
}*/