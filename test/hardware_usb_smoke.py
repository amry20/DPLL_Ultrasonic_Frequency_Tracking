"""Read-only USB checks after upload. Requires pyserial; does not start sonication."""
import argparse
import serial,struct,time

def frame(op,data=b''):
    return struct.pack('<BHHHB',0xaa,op,0,len(data)+1,0xbb)+data+bytes([(-sum(data))&255])

def receive(port,opcode,expected=1):
    end=time.monotonic()+2; buf=bytearray(); packets=[]
    while time.monotonic()<end:
        buf.extend(port.read(port.in_waiting or 1))
        while len(buf)>=8:
            if buf[0]!=0xaa or buf[7]!=0xbb:
                del buf[0]; continue
            _,op,addr,length,_=struct.unpack('<BHHHB',buf[:8])
            if not 1<=length<=512:
                del buf[0]; continue
            if len(buf)<8+length: break
            payload=bytes(buf[8:8+length]); del buf[:8+length]
            if sum(payload)&255: raise AssertionError('bad response checksum')
            if op==opcode:
                packets.append((addr,payload[:-1]))
                if len(packets)==expected: return packets
    raise AssertionError('response timeout for '+hex(opcode))
parser = argparse.ArgumentParser()
parser.add_argument('--port', default='COM20')
args = parser.parse_args()
deadline = time.monotonic() + 10
while True:
    try:
        connection = serial.Serial(args.port,115200,timeout=.02,write_timeout=1)
        break
    except serial.SerialException:
        if time.monotonic() >= deadline:
            raise
        time.sleep(.1)
with connection as port:
    port.reset_input_buffer()
    for byte in frame(1):
        port.write(bytes([byte])); time.sleep(.002)
    version=receive(port,1)[0][1].rstrip(bytes([0])).decode()
    assert version=='DPLL v2.0.1',version
    print('PASS USB fragmented GET_VERSION:',version)
    port.write(frame(0x7ffe,bytes(range(255))*2+b'!'))
    port.write(frame(1))
    assert receive(port,1)[0][1].startswith(b'DPLL v2.0.1')
    print('PASS USB 511-byte data frame followed by valid command')
    port.write(struct.pack('<BHHHB',0xaa,0x7ffe,0,12,0xbb)+bytes([17]))
    time.sleep(.15)
    port.write(frame(1))
    assert receive(port,1)[0][1].startswith(b'DPLL v2.0.1')
    print('PASS USB parser recovery after truncated frame')
    port.write(frame(0x0047)); source=receive(port,0x0047)[0][1][0]
    print('Phase source:',source)
    port.write(frame(0x0048)); snapshots=receive(port,0x0049,2)
    for addr,payload in snapshots:
        header=struct.unpack('<BBHIfff',payload[:20])
        assert header[2]==240 and len(payload)==500
        print('PASS USB ADC snapshot:',{'channel':header[0],'samples':header[2],'sampleRate':header[3],'driveHz':header[4]})
