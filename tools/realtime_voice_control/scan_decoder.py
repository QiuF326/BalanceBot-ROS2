"""Bounded CDR1 LaserScan decoding, without allocating an unused intensities array."""
import struct
from types import SimpleNamespace


def decode_scan(data):
    if not isinstance(data,(bytes,bytearray,memoryview)) or not 60<=len(data)<=165000:
        raise ValueError('雷达帧长度无效')
    if bytes(data[:4]) not in (b'\x00\x01\x00\x00',b'\x00\x00\x00\x00'):
        raise ValueError('不支持的雷达 CDR 编码')
    endian='<' if data[1]==1 else '>'
    offset=4
    def take(fmt):
        nonlocal offset
        size=struct.calcsize(fmt)
        if offset+size>len(data):raise ValueError('雷达帧被截断')
        result=struct.unpack_from(endian+fmt,data,offset);offset+=size
        return result
    sec,nsec,length=take('iII')
    if nsec>=1000000000 or not 1<=length<=256 or offset+length>len(data):
        raise ValueError('雷达帧头无效')
    frame=bytes(data[offset:offset+length]);offset+=length
    if frame[-1]!=0:raise ValueError('雷达坐标名未正确结束')
    frame=frame[:-1].decode('utf-8',errors='strict')
    offset=4+((offset-4+3)//4)*4
    angle_min,angle_max,step,time_increment,scan_time,low,high=take('7f')
    count,=take('I')
    if not 1<=count<=20000 or offset+count*4+4>len(data):
        raise ValueError('雷达测距数组无效')
    ranges=take(str(count)+'f')
    intensity_count,=take('I')
    remaining=len(data)-offset
    if intensity_count>20000 or not intensity_count*4<=remaining<=intensity_count*4+3:
        raise ValueError('雷达强度数组长度无效')
    return SimpleNamespace(header=SimpleNamespace(frame_id=frame,stamp=SimpleNamespace(sec=sec,nanosec=nsec)),
        angle_min=angle_min,angle_max=angle_max,angle_increment=step,time_increment=time_increment,
        scan_time=scan_time,range_min=low,range_max=high,ranges=ranges)
