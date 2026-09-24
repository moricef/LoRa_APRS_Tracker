// Dummy Arduino.h for native tests
#ifndef ARDUINO_H
#define ARDUINO_H

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <string>
#include <chrono>
#include <cstdlib>

unsigned long millis();

typedef uint8_t byte;

#ifndef TWO_PI
#define TWO_PI 6.283185307179586476925286766559
#endif

inline double radians(double deg) { return deg * 0.017453292519943295769236907684886; }
inline double degrees(double rad) { return rad * 57.295779513082320876798154814105; }
inline double sq(double x) { return x * x; }

// Minimal Arduino String mock for station_utils.h
class String {
public:
    std::string s;
    String() {}
    String(const char* c) : s(c ? c : "") {}
    String(const std::string& str) : s(str) {}
    String(int value) : s(std::to_string(value)) {}
    const char* c_str() const { return s.c_str(); }
    int length() const { return static_cast<int>(s.length()); }
    bool isEmpty() const { return s.empty(); }
    char charAt(int index) const { return index >= 0 && index < length() ? s[index] : 0; }
    char operator[](int index) const { return charAt(index); }
    int indexOf(char c, int from = 0) const {
        auto pos = s.find(c, from);
        return pos == std::string::npos ? -1 : static_cast<int>(pos);
    }
    int indexOf(const char* value) const {
        auto pos = s.find(value);
        return pos == std::string::npos ? -1 : static_cast<int>(pos);
    }
    String substring(int from, int to) const { return s.substr(from, to - from); }
    String substring(int from) const { return s.substr(from); }
    void remove(int from) { s.erase(from); }
    void setCharAt(int index, char value) { s[index] = value; }
    long toInt() const { return std::strtol(s.c_str(), nullptr, 10); }
    String& operator+=(char value) { s += value; return *this; }
    String& operator+=(const String& value) { s += value.s; return *this; }
    String& operator+=(int value) { s += std::to_string(value); return *this; }
    friend String operator+(String lhs, const String& rhs) { lhs += rhs; return lhs; }
    friend String operator+(String lhs, char rhs) { lhs += rhs; return lhs; }
    friend String operator+(char lhs, const String& rhs) { String result; result += lhs; result += rhs; return result; }
    bool operator==(const String& rhs) const { return s == rhs.s; }
};

#endif
