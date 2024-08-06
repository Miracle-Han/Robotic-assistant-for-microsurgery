#ifndef SERIAL_READER_H
#define SERIAL_READER_H

#include <boost/asio.hpp>
#include <string>
#include <vector>

using namespace boost::asio;
using namespace std;

class SerialReader {
public:
    SerialReader(const string& port_name, int baud_rate);
    vector<int> readLineAsIntArray();
    double calculateAverage(const vector<int>& array);

private:
    io_service io;
    serial_port serial;

    vector<int> splitStringToIntArray(const string& str);
};

#endif // SERIAL_READER_H
