"""Exclusive Windows COM transport. Opening is explicit; never toggles DTR/RTS."""
import ctypes as C
from ctypes import wintypes as W
import os
import re


class SerialPort:
    def __init__(self, port):
        if os.name != 'nt' or not re.fullmatch(r'COM[1-9][0-9]*', port.upper()):
            raise ValueError('Cần tên cổng Windows, ví dụ COM4.')
        k = C.WinDLL('kernel32', use_last_error=True)
        k.CreateFileW.argtypes = [W.LPCWSTR, W.DWORD, W.DWORD, C.c_void_p, W.DWORD, W.DWORD, W.HANDLE]
        k.CreateFileW.restype = W.HANDLE
        k.CloseHandle.argtypes = [W.HANDLE]
        k.ReadFile.argtypes = [W.HANDLE, C.c_void_p, W.DWORD, C.POINTER(W.DWORD), C.c_void_p]
        k.WriteFile.argtypes = k.ReadFile.argtypes
        self.k, self.h = k, None
        h = k.CreateFileW('\\\\.\\'+port.upper(), 0xC0000000, 0, None, 3, 0, None)
        if h == C.c_void_p(-1).value:
            raise OSError('Không mở được '+port+'. Đóng UART_COM4.cmd và công cụ serial khác trước.')
        self.h = h
        class DCB(C.Structure):
            _fields_ = [('length', W.DWORD), ('baud', W.DWORD), ('flags', W.DWORD),
                        ('reserved', W.WORD), ('xonlim', W.WORD), ('xofflim', W.WORD),
                        ('bytesize', W.BYTE), ('parity', W.BYTE), ('stopbits', W.BYTE),
                        ('xon', C.c_char), ('xoff', C.c_char), ('error', C.c_char),
                        ('eof', C.c_char), ('event', C.c_char), ('reserved1', W.WORD)]
        class Timeouts(C.Structure):
            _fields_ = [(n, W.DWORD) for n in ['interval', 'read_mult', 'read_const', 'write_mult', 'write_const']]
        k.BuildCommDCBW.argtypes = [W.LPCWSTR, C.POINTER(DCB)]
        k.SetCommState.argtypes = [W.HANDLE, C.POINTER(DCB)]
        k.SetCommTimeouts.argtypes = [W.HANDLE, C.POINTER(Timeouts)]
        k.PurgeComm.argtypes = [W.HANDLE, W.DWORD]
        try:
            d = DCB(); d.length = C.sizeof(DCB)
            self._ok(k.BuildCommDCBW('baud=115200 parity=N data=8 stop=1 dtr=off rts=off', C.byref(d)))
            self._ok(k.SetCommState(h, C.byref(d)))
            self._ok(k.SetCommTimeouts(h, C.byref(Timeouts(0xFFFFFFFF, 0, 0, 0, 100))))
            self._ok(k.PurgeComm(h, 0x000C))
        except Exception:
            self.close()
            raise

    def _ok(self, value):
        if not value:
            raise C.WinError(C.get_last_error())

    def read(self):
        buf = C.create_string_buffer(4096); count = W.DWORD()
        self._ok(self.k.ReadFile(self.h, buf, len(buf), C.byref(count), None))
        return buf.raw[:count.value]

    def write(self, data):
        count = W.DWORD(); buf = C.create_string_buffer(data)
        self._ok(self.k.WriteFile(self.h, buf, len(data), C.byref(count), None))
        if count.value != len(data):
            raise OSError('UART chỉ gửi được một phần khung.')

    def close(self):
        if self.h is not None:
            self.k.CloseHandle(self.h)
            self.h = None
