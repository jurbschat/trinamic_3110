#include <iostream>
#include <fstream>
#include <thread>
#include <boost/asio.hpp>
#include <string>

class SimpleSerial
{
public:
	/**
	 * Constructor.
	 * \param port device name, example "/dev/ttyUSB0" or "COM4"
	 * \param baud_rate communication speed, example 9600 or 115200
	 * \throws boost::system::system_error if cannot open the
	 * serial device
	 */
	SimpleSerial(std::string port, unsigned int baud_rate)
			: io(), serial(io,port)
	{
		serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
	}

	/**
	 * Write a string to the serial device.
	 * \param s string to write
	 * \throws boost::system::system_error on failure
	 */
	void writeString(std::string s)
	{
		boost::asio::write(serial,boost::asio::buffer(s.c_str(),s.size()));
	}

	/**
	 * Blocks until a line is received from the serial device.
	 * Eventual '\n' or '\r\n' characters at the end of the string are removed.
	 * \return a string containing the received line
	 * \throws boost::system::system_error on failure
	 */
	std::string readLine()
	{
		//Reading data char by char, code is optimized for simplicity, not speed
		using namespace boost;
		char c;
		std::string result;
		for(;;)
		{
			asio::read(serial,asio::buffer(&c,1));
			switch(c)
			{
				case '\r':
					break;
				case '\n':
					return result;
				default:
					result+=c;
			}
		}
	}

	std::string read(std::size_t amount) {
		using namespace boost;
		char result[9];
		asio::read(serial, asio::buffer(result, std::size(result)));
		return std::string(result);
	}

private:
	boost::asio::io_service io;
	boost::asio::serial_port serial;
};

int main()
{
	SimpleSerial serial("/dev/ttyUSB0", 9600);
	unsigned char move_cmd[] = {0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x03, 0xe8, 0xed};
	unsigned char stop_cmd[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04};
	{
		auto cmd = std::string((char*)&move_cmd[0], std::size(move_cmd));
		serial.writeString(cmd);
		std::string response = serial.read(9);
		std::cout << response << std::endl;
	}
	std::this_thread::sleep_for(std::chrono::seconds(2));
	{
		auto cmd = std::string((char*)&stop_cmd[0], std::size(stop_cmd));
		serial.writeString(cmd);
		std::string response = serial.read(9);
		std::cout << response << std::endl;
	}

	return 0;
}