#include "SerialReader.h"
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <numeric>

SerialReader::SerialReader(const string& port_name, int baud_rate)
    : io(), serial(io, port_name) {
    // 设置串口参数
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
        // 从串口读取一个字符
        boost::asio::read(serial, buffer(&c, 1), ec);

        // 如果读取出错，打印错误信息并返回默认数组
        if (ec) {
            cerr << "Read error: " << ec.message() << endl;
            return vector<int>(8, 0); // 返回一个填充了0的默认数组
        }

        // 如果读取到换行符，解析当前行并返回结果
        if (c == '\n') {
            return splitStringToIntArray(line); // 返回解析后的整数数组
        } else {
            line += c;
        }
    }
}

vector<int> SerialReader::splitStringToIntArray(const string& str) {
    vector<int> tokens(8, 0);  // 初始化为 8 个 0
    stringstream ss(str);
    string token;
    int index = 0;

    while (getline(ss, token, ' ') && index < 8) {
        if (!token.empty()) {
            try {
                // 将字符串转换为整数并存储到数组中
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

// 计算数组平均值的函数
double SerialReader::calculateAverage(const vector<int>& array) {
    if (array.empty()) return 0.0;
    double sum = accumulate(array.begin(), array.end(), 0);
    return sum / array.size();
}