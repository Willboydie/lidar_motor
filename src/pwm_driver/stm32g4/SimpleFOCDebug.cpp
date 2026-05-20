#include "../../hardware_api.h"

Print* SimpleFOCDebug::_debugPrint = &Serial;

void SimpleFOCDebug::enable(Print* debugPrint) {
  _debugPrint = debugPrint ? debugPrint : &Serial;
}

void SimpleFOCDebug::println(const __FlashStringHelper* msg) {
  if (_debugPrint) _debugPrint->println(msg);
}

void SimpleFOCDebug::println(const StringSumHelper msg) {
  if (_debugPrint) _debugPrint->println(msg);
}

void SimpleFOCDebug::println(const char* msg) {
  if (_debugPrint) _debugPrint->println(msg);
}

void SimpleFOCDebug::println(const __FlashStringHelper* msg, float val) {
  if (_debugPrint) {
    _debugPrint->print(msg);
    _debugPrint->println(val);
  }
}

void SimpleFOCDebug::println(const char* msg, float val) {
  if (_debugPrint) {
    _debugPrint->print(msg);
    _debugPrint->println(val);
  }
}

void SimpleFOCDebug::println(const __FlashStringHelper* msg, int val) {
  if (_debugPrint) {
    _debugPrint->print(msg);
    _debugPrint->println(val);
  }
}

void SimpleFOCDebug::println(const char* msg, int val) {
  if (_debugPrint) {
    _debugPrint->print(msg);
    _debugPrint->println(val);
  }
}

void SimpleFOCDebug::println(const char* msg, char val) {
  if (_debugPrint) {
    _debugPrint->print(msg);
    _debugPrint->println(val);
  }
}

void SimpleFOCDebug::println() {
  if (_debugPrint) _debugPrint->println();
}

void SimpleFOCDebug::println(int val) {
  if (_debugPrint) _debugPrint->println(val);
}

void SimpleFOCDebug::println(float val) {
  if (_debugPrint) _debugPrint->println(val);
}

void SimpleFOCDebug::print(const char* msg) {
  if (_debugPrint) _debugPrint->print(msg);
}

void SimpleFOCDebug::print(const __FlashStringHelper* msg) {
  if (_debugPrint) _debugPrint->print(msg);
}

void SimpleFOCDebug::print(const StringSumHelper msg) {
  if (_debugPrint) _debugPrint->print(msg);
}

void SimpleFOCDebug::print(int val) {
  if (_debugPrint) _debugPrint->print(val);
}

void SimpleFOCDebug::print(float val) {
  if (_debugPrint) _debugPrint->print(val);
}
