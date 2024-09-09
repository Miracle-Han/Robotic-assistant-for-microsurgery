#include "SerialReader.h"
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <numeric>

SerialReader::SerialReader(const string& port_name, int baud_rate)
    : io(), serial(io, port_name) {
    // Setting serial port parameters
    serial.set_option(serial_port_base::baud_rate(baud_rate));
    serial.set_option(serial_port_base::character_size(8));
    serial.set_option(serial_port_base::parity(serial_port_base::parity::none));
    serial.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
}

vector<int> SerialReader::readLineAsIntArray() {
    string line;
    boost::system::error_code ec;

    while (true) {
        char c;
        // Read a character from the serial port
        boost::asio::read(serial, buffer(&c, 1), ec);

        if (ec) {
            cerr << "Read error: " << ec.message() << endl;
            return vector<int>(8, 0);
        }

        // If a newline is read, the current line is parsed and the result is returned
        if (c == '\n') {
            return splitStringToIntArray(line);
        } else {
            line += c;
        }
    }
}

vector<int> SerialReader::splitStringToIntArray(const string& str) {
    vector<int> tokens(8, 0);
    stringstream ss(str);
    string token;
    int index = 0;

    while (getline(ss, token, ' ') && index < 8) {
        if (!token.empty()) {
            try {
                // Converts a string to an integer and stores it in an array
                tokens[index] = stoi(token);
                index++;
            } catch (const invalid_argument& e) {
                cerr << "Invalid argument: " << e.what() << " for token: " << token << endl;
            } catch (const out_of_range& e) {
                cerr << "Out of range: " << e.what() << " for token: " << token << endl;
            }
        }
    }

    return tokens;
}

// calculates the average value of an array
double SerialReader::calculateAverage(const vector<int>& array) {
    if (array.empty()) return 0.0;
    double sum = accumulate(array.begin(), array.end(), 0);
    return sum / array.size();
}