#pragma once
//
// Created by urbschaj on 2019-09-30.
//
#include <string>
#include <cstdint>
#include <type_traits>
#include <experimental/optional>
//#include <unordered_map>
#include <boost/asio.hpp>



namespace TMCM {

	template<typename T>
	using optional = std::experimental::optional<T>;

	class Exception : public std::exception {
	public:
		Exception(std::string error);
		const char* what() const noexcept;
	private:
		std::string error;
	};

	/*
	 * parsing helpers
	 */
	uint8_t BuildChecksum(const std::array<uint8_t, 8>& data);

	template<typename T>
	using IsFittingIntegral = typename std::enable_if<std::is_integral<T>::value && sizeof(T) == 4>::type;

	//TODO: does this work on arm?
	template<typename T, typename = IsFittingIntegral<T>>
	std::array<uint8_t, 4> BuildPayload(T value) {
		uint8_t byte1 =  value & 0x000000ff;
		uint8_t byte2 = (value & 0x0000ff00) >> 8;
		uint8_t byte3 = (value & 0x00ff0000) >> 16;
		uint8_t byte4 = (value & 0xff000000) >> 24;
		std::array<uint8_t, 4> out {byte4, byte3, byte2, byte1};
		return out;
	}

	//TODO: does this work on arm?
	template<typename T, typename = IsFittingIntegral<T>>
	T ParsePayload(const std::array<uint8_t, 4>& payload) {
		T out;
		out = (payload[0] << 24) | (payload[1] << 16) | (payload[2] << 8) | payload[3];
		return out;
	}

	template<size_t start, size_t count, typename T>
	std::array<typename T::value_type, count> SliceBuffer(const T& buffer) {
		static_assert(std::tuple_size<T>::value >= start + count, "invalid sizes for slice");
		std::array<typename T::value_type, count> out;
		std::copy(buffer.begin() + start, buffer.begin() + start + count, out.begin());
		return out;
	}

	/*
	 * everything we send/receive is always 9 bytes
	 */
	using CommandResponseBuffer = std::array<uint8_t, 9>;

	/*
	 * all the nessesary enums and defines to have better names when building commands
	 */
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
		SetAxisParameter=5,         //  (SAP) set axis paramater (p)
		GetAxisParameter=6,         //  (GAP) get axis paramater (p)
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
		namespace AxisParamaters {
			enum class Type {
				TargetPosition=0,
				ActualPosition=1,
				TargetSpeed=2,
				ActualSpeed=3,
				MaxSpeed=4,
				MaxAcceleration=5,
				MaxCurrent=6,
				StandbyCurrent=7,
				PositionReached=8,
				HomeSwitchState=9,
				RightLimitSwitch=10,
				LeftLimitSwitch=11,
				RightLimitSwitchDisable=12,
				LeftLimitSwitchDisable=13,
				MinimumSpeed=130,
				ActualAcceleration=135,
				RampMode=138,
				MicrostepResolution=140,
				SoftStopFlag=149,
				EndSwitchPowerDown=150,
				RampDivisor=153,
				PulseDivisor=154,
				StepInterpolation=160,
				ReferenceSearchMode=193,
				ReferenceSearchSpeed=194,
				ReferenceSwitchSpeed=195,
				EndSwitchDistance=196,
				LastReferencePosition=197,
				FreeWheeling=204,
				ActualLoad=206,
				PowerDownDelay=214,
				StepMode=254

			};
			enum class Value {

			};
		}
		namespace MoveToPosition {
			enum class Type {
				Absolute=0,
				Relative=1,
				Coord=2
			};
		}
		namespace ReferenceSearch {
			enum class Type {
				Start = 0,
				Stop = 1,
				Status = 2
			};
		}
		namespace SetIO {
			const CommandTypes::VALUE_TYPE VALID_OUTPUT_BANK = 2;
			enum Type {

			};
			enum Value {};
		}
	}

	/*
	 * two helper classes that provide utility functions to build commands and parse responses
	 */
	class TMCMCommand
	{
	public:
		template<typename T, typename = IsFittingIntegral<T>>
		TMCMCommand(uint8_t module, CommandCodes command, uint8_t type, uint8_t motor, T payload)
				: TMCMCommand(module, command, type, motor, BuildPayload(payload))
		{}

		TMCMCommand(uint8_t module, CommandCodes command, uint8_t type, uint8_t motor, std::array<uint8_t, 4> payload);
		const CommandResponseBuffer& GetData() const;
		std::string to_string() const;
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

		TMCMResoponse(uint8_t replyAddress, uint8_t moduleAddress, uint8_t status, uint8_t command, std::array<uint8_t, 4> payload, CommandResponseBuffer raw);
	public:
		static optional<TMCMResoponse> ParseResponse(const CommandResponseBuffer& response);
		uint8_t GetReplyAddress() const;
		uint8_t GetModuleAddress() const;
		ReturnCodes GetStatus() const;
		uint8_t GetCmmand() const;
		int32_t PayloadAsInt();
		uint32_t PayloadAsUInt();
		std::array<uint8_t, 4> GetPayload() const;
		CommandResponseBuffer GetRaw() const;
		std::string to_string() const;
	};

	/*
	 * provides the actual serial communication using asio
	 */
	class SerialInterface
	{
	public:
		SerialInterface(std::string serialDevice, int32_t baudRate, float timeout);
		~SerialInterface();

		TMCMResoponse writeRead(const TMCMCommand& command);
		void write(const TMCMCommand& command);
		void write(const CommandResponseBuffer& command);
		optional<CommandResponseBuffer> writeRead(const CommandResponseBuffer& command);
		void write(const std::string& s);
		std::string read();

	private:
		boost::asio::io_service io;
		boost::asio::serial_port port;
		boost::asio::deadline_timer timer;
		boost::posix_time::time_duration timeout;
	};

	/*
	 * simple helper types to have better paramater names than f(1, 2, 3) => f(Module(1), Motor(2), 12)
	 */
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
	using IO = Motor;

	namespace Builder {
		TMCMCommand RotateRight(Module module, Motor motor, uint32_t velocity);
		TMCMCommand RotateLeft(Module module, Motor motor, uint32_t velocity);
		TMCMCommand StopMotor(Module module, Motor motor);
		TMCMCommand SetGlobalParameter(Module module, TypeParams::GlobalParamaters::Type type, TypeParams::GlobalParamaters::Value value, Bank bank = Bank(0));
		TMCMCommand GetGlobalParameter(Module module, TypeParams::GlobalParamaters::Type type, Bank bank = Bank(0));
		TMCMCommand SetAxisParemater(Module module, TypeParams::AxisParamaters::Type type, Motor motor, int32_t data);
		TMCMCommand GetAxisParamater(Module module, TypeParams::AxisParamaters::Type type, Motor motor);
		TMCMCommand MoveToPosition(Module module, Motor motor, int32_t pos);
		TMCMCommand ReferenceSearch(Module module, Motor motor);
		TMCMCommand SetOutput(int16_t module, IO ioPort, bool state);
		TMCMCommand GetInput(int16_t module, IO ioPort, Bank bank);
		/*
		TMCMCommand MoveToPosition(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand SetAxisParemater(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand GetAxisParamater(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand StoreAxisParamater(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand RestoreAxisParamater(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand SetGlobalParamater(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand GetGlobalParameter(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand StoreGlobalParamater(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand RestoreGlobalParamater(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand ReferenceSearch(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand SetOutput(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand GetInput(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand CALC(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand COMP(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand JC(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand JA(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand CSUB(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand RSUB(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand EnableInterrupt(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand DisableInterrupt(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand WAIT(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand STOP(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand StoreCoordinate(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand GetCordinate(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand CaptureCoordinate(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand CALCX(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand AAP(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand AGP(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand ClearErrorFlag(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand VECT(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand RETI(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		TMCMCommand ACO(int16_t module, _DAT_TYPE_, uint8_t motor/bank, data) {}
		 */
		TMCMCommand Restart(Module module);
		TMCMCommand GetFirmwareVersion(Module module);
	}

	/*
	 * the class that will be used by the tmcm users to interface with the device
	 */
	class ControllerInterface
	{
	public:
		ControllerInterface(std::string serialDevice, int32_t baudRate, const std::vector<int32_t>& modules);
		~ControllerInterface();
		TMCMResoponse writeRead(const TMCMCommand& command);

	private:
		void SetupConnection(std::string serialDevice, int32_t requestedRate, const std::vector<int32_t>& modules);

		std::unique_ptr<SerialInterface> si;
		std::mutex mtx;
		//std::unordered_map<int32_t, int32_t> activeModules;
	};

	class Core {
	public:
		Core();
		~Core();
		void Init(std::string serialDevice, int32_t baudRate, const std::vector<int32_t>& modules);
		std::unique_ptr<ControllerInterface>& GetInterface();
	private:
		std::unique_ptr<ControllerInterface> tmcmInterface;
	};

	Core& GetCore();
}
