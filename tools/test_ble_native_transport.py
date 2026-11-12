#!/usr/bin/env python3
"""Exercise the firmware BLE parsers and notifier on a host PC.

Uses the actual functions from ble_utils.cpp and the real KISS codec;
FreeRTOS and the NimBLE notification API are test doubles. Radio operation
and controller lifecycle still require validation on the T-Deck.
"""
from pathlib import Path
import tempfile, subprocess
root=Path(__file__).resolve().parents[1]
src=(root/'src/ble_utils.cpp').read_text()
work=tempfile.TemporaryDirectory(prefix='idf5-ble-test-')
d=Path(work.name)
mock=(root/'test/Arduino.h').read_text().replace('    void remove(int from) { s.erase(from); }','    void remove(int from) { s.erase(from); }\n    void remove(int from, int count) { s.erase(from,count); }')
(d/'Arduino.h').write_text(mock)
parsers=src[src.index('static void resetTnc2Input()'):src.index('static void clearConnection()')]
a=src.index('    static void notifyBytes(');b=src.index('    void txToPhoneOverBLE(',a);notify=src[a:b]
a=src.index('static bool stopHost()'); b=src.index('namespace BLE_Utils {',a)
shutdown=src[a:b]
a=src.index('static void clearConnection()'); b=src.index('static int gapEvent(',a)
clear=src[a:b]
preamble=r'''
#include <cassert>
#include <atomic>
#include <vector>
#include <array>
#include <algorithm>
#include <cstring>
#include "Arduino.h"
#include "kiss_utils.h"
#include "mock_esp_log.h"
#define BLE_CHUNK_SIZE 512
#define MAX_KISS_BUFFER 1024
#define MAX_TNC2_LINE 512
#define TNC2_QUEUE_CAPACITY 8
#define BLE_HS_CONN_HANDLE_NONE 0xffff
#define pdTRUE 1
#define portMAX_DELAY 0xffffffff
#define pdMS_TO_TICKS(x) (x)
using SemaphoreHandle_t=void*;
static void* xSemaphoreCreateMutex() {static int mutex;return &mutex;}
static void* hostExited=(void*)2;
static std::vector<int> shutdownCalls;
static int xSemaphoreTake(void* handle,int) {
 if(handle==hostExited) shutdownCalls.push_back(2);
 return pdTRUE;
}
static void xSemaphoreGive(void*) {}
static const char* TAG="BLE_TEST";
static SemaphoreHandle_t tnc2QueueMutex=0;
static String tnc2Queue[TNC2_QUEUE_CAPACITY],kissSerialBuffer,tnc2SerialBuffer;
static uint8_t tnc2QueueHead=0,tnc2QueueTail=0,tnc2QueueCount=0;
static bool tnc2DiscardUntilDelimiter=false;
static bool bleInitialized=true,bleHostStarted=true;
static std::atomic<bool> bleSynced{true};
static bool bluetoothConnected=true;
static char peerAddress[18]={};
#define ESP_OK 0
using esp_err_t=int;
static int stopError=0;
static int nimble_port_stop() { shutdownCalls.push_back(1);return stopError; }
static int nimble_port_deinit() { shutdownCalls.push_back(3);return ESP_OK; }
static const char* esp_err_to_name(int) {return "test error";}
static std::atomic<bool> bleStopping{false},bleSubscribed{true};
static std::atomic<uint16_t> bleConnection{1};
static std::atomic<uint32_t> bleSession{1};
static uint8_t txValue[512];
static uint16_t txValueLength=0,txValueHandle=16,mtu=23;
static int stateMux;
#define portENTER_CRITICAL(x) ((void)0)
#define portEXIT_CRITICAL(x) ((void)0)
struct LifecycleLock {};
struct os_mbuf { std::vector<uint8_t> data; };
static std::vector<std::vector<uint8_t>> notifications;
static int buffers=0,notifyError=0;
static bool dropAfterFirst=false,reconnectAfterFirst=false;
static uint16_t ble_att_mtu(uint16_t) { return mtu; }
static os_mbuf* ble_hs_mbuf_from_flat(const uint8_t* data,size_t len) {
 buffers++;return new os_mbuf{std::vector<uint8_t>(data,data+len)};
}
static int ble_gatts_notify_custom(uint16_t,uint16_t,os_mbuf* p) {
 notifications.push_back(p->data);delete p;buffers--;return notifyError;
}
static void delay(int) {
 if (dropAfterFirst && notifications.size()==1) bleConnection=BLE_HS_CONN_HANDLE_NONE;
 if (reconnectAfterFirst && notifications.size()==1) bleSession++;
}
'''
main=r'''
static void feedT(const String& s) { receiveTnc2Bytes((const uint8_t*)s.c_str(),s.length()); }
static void feedK(const String& s) { receiveKissBytes((const uint8_t*)s.c_str(),s.length()); }
static String pop() { String s; assert(dequeueTnc2Line(s));return s; }
static void empty() { String s; assert(!dequeueTnc2Line(s)); }
int main() {
 resetTnc2Input();
 String alpha="F4MLV-GS>APLRG1,WIDE1-1:extended identity";
 feedT(alpha.substring(0,13));empty();feedT(alpha.substring(13)+"\r\n");assert(pop()==alpha);empty();
 String classic="F4MLV-7>APLRG1,WIDE1-1:classic";
 feedT(alpha+"\n"+classic+"\n");assert(pop()==alpha);assert(pop()==classic);empty();
 feedT("invalid\n");empty();
 String huge; for(int i=0;i<600;i++)huge+='X';
 feedT(huge+"\n"+alpha+"\n");assert(pop()==alpha);empty();
 feedT(alpha.substring(0,10));resetTnc2Input();feedT(classic+"\n");assert(pop()==classic);empty();
 for(int i=0;i<9;i++)feedT(alpha+"\n");
 for(int i=0;i<8;i++)assert(pop()==alpha);empty();
 String wire=KISS_Utils::encodeKISS(classic);assert(!wire.isEmpty());
 feedK(wire.substring(0,7));empty();feedK(wire.substring(7));assert(pop()==classic);empty();
 feedK(wire+wire);assert(pop()==classic);assert(pop()==classic);empty();
 String escaped=classic+(char)KissChar::FEND+(char)KissChar::FESC;
 feedK(KISS_Utils::encodeKISS(escaped));assert(pop()==escaped);empty();
 String control=wire;control.setCharAt(1,1);feedK(control);empty();
 assert(KISS_Utils::encodeKISS(alpha).isEmpty());
 std::array<uint8_t,600> bytes{};for(size_t i=0;i<bytes.size();i++)bytes[i]=i;
 for(uint16_t testedMtu:{23,256,517}) {
  mtu=testedMtu;notifications.clear();notifyBytes(bytes.data(),bytes.size());
  std::vector<uint8_t> received;for(const auto& n:notifications) {
   assert(n.size()<=std::min<size_t>(mtu-3,512));received.insert(received.end(),n.begin(),n.end());
  }
  assert(received==std::vector<uint8_t>(bytes.begin(),bytes.end()));assert(buffers==0);
 }
 notifications.clear();bleSubscribed=false;notifyBytes(bytes.data(),bytes.size());assert(notifications.empty());bleSubscribed=true;
 mtu=23;notifications.clear();dropAfterFirst=true;notifyBytes(bytes.data(),bytes.size());assert(notifications.size()==1);dropAfterFirst=false;bleConnection=1;
 notifications.clear();reconnectAfterFirst=true;notifyBytes(bytes.data(),bytes.size());assert(notifications.size()==1);reconnectAfterFirst=false;
 notifications.clear();notifyError=1;notifyBytes(bytes.data(),bytes.size());assert(notifications.size()==1 && buffers==0);
 feedT(alpha+"\n"+alpha.substring(0,9));feedK(wire.substring(0,7));
 bleHostStarted=true;bleInitialized=true;shutdownCalls.clear();
 assert(stopHost());assert(shutdownCalls==std::vector<int>({1,2,3}));
 assert(!bleInitialized && !bleHostStarted && !bluetoothConnected && !bleSynced);
 assert(bleConnection==BLE_HS_CONN_HANDLE_NONE);empty();
 assert(tnc2SerialBuffer.isEmpty() && kissSerialBuffer.isEmpty());
 assert(stopHost());assert(shutdownCalls.size()==3);
 bleInitialized=true;bleHostStarted=true;stopError=17;shutdownCalls.clear();
 assert(!stopHost());assert(bleInitialized && bleHostStarted && bleStopping);
 assert(shutdownCalls==std::vector<int>({1}));
 stopError=0;assert(stopHost());assert(shutdownCalls==std::vector<int>({1,1,2,3}));
 bleInitialized=true;bleHostStarted=false;shutdownCalls.clear();
 assert(stopHost());assert(shutdownCalls==std::vector<int>({3}));
 puts("PASS: lifecycle stop/wait/deinit ordering, idempotent stop, failed-stop retry, cleanup before host start");
 puts("PASS: TNC2/KISS fragmentation, ordering, escapes, overflow recovery, MTU 23/256/517, unsubscribe, disconnect/reconnect and notification failure");
}
'''
(d/'test.cpp').write_text(preamble+parsers+clear+shutdown+notify+main)
cmd=['g++','-std=c++17','-DUNIT_TEST','-I'+str(d),'-I'+str(root/'test'),'-I'+str(root/'include'),str(d/'test.cpp'),str(root/'src/kiss_utils.cpp'),'-o',str(d/'test')]
r=subprocess.run(cmd,capture_output=True,text=True)
if r.returncode: print(r.stderr[-3500:]);raise SystemExit(r.returncode)
subprocess.run([str(d/'test')],check=True)

# Exercise the real Arduino startup hook selected by the firmware source.
# Only the chip configuration headers are mocked for this host build.
core=Path.home()/'.platformio/packages/framework-arduinoespressif32/cores/esp32'
(d/'soc').mkdir()
(d/'soc/soc_caps.h').write_text('#pragma once\n#define CONFIG_SOC_BLE_SUPPORTED 1\n')
(d/'sdkconfig.h').write_text('#pragma once\n#define CONFIG_BT_CONTROLLER_ENABLED 1\n')
hook='\n'.join(line for line in src.splitlines() if line.strip()=='#include <esp32-hal-alloc-ble-mem.h>')
(d/'startup.cpp').write_text('#include <cassert>\nextern "C" { bool _bleLibraryInUse=false; }\n'+hook+'\nint main() { assert(_bleLibraryInUse); }\n')
subprocess.run(['g++','-I'+str(d),'-I'+str(core),str(d/'startup.cpp'),'-o',str(d/'startup')],check=True)
subprocess.run([str(d/'startup')],check=True)
print('PASS: native BLE declares its memory use before Arduino startup')
