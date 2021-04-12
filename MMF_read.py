import sys
import mmap
import time
from ctypes import *
from ctypes.wintypes import BOOL, DWORD, HANDLE, LPCWSTR, LPCVOID, LPVOID
 
 
class Mmf:
    def __init__(self, nameOfMmf):
  
 
        if(type(nameOfMmf) != type("")):
            print("nameOfMmf 인자가 문자열이 아닙니다.")
            return 1
 
        
    
        
        
        self.FILE_MAP_ALL_ACCESS = 0x000F001F # 플래그의 상수값
        self.INVALID_HANDLE_VALUE = -1 # 시스템 페이지를 사용하기위한 파일디스크립터(INVALID_FILE_HANDLER)
        self.SHMEMSIZE = 0x100 # 공유메모리 크기
        self.PAGE_READWRITE = 0x04 # 플래그의 상수값
        
        self.FILE_MAP_READ = 0x0004 # 플래그의 상수값
        
 
        self.kernel32_dll = windll.kernel32
        self.msvcrt_dll = cdll.msvcrt
 
        self.create_file_mapping_func = self.kernel32_dll.CreateFileMappingW
        self.create_file_mapping_func.argtypes = (HANDLE, LPVOID, DWORD, DWORD, DWORD, LPCWSTR)
        self.create_file_mapping_func.restype = HANDLE
        
        self.open_file_mapping_func = self.kernel32_dll.OpenFileMappingW
        self.open_file_mapping_func.argtypes = (DWORD, BOOL, LPCWSTR)
        self.open_file_mapping_func.restype = HANDLE
 
 
        self.map_view_of_file_func = self.kernel32_dll.MapViewOfFile
        self.map_view_of_file_func.argtypes = (HANDLE, DWORD, DWORD, DWORD, c_ulonglong)
        self.map_view_of_file_func.restype = LPVOID
 
        self.memcpy_func = self.msvcrt_dll.memcpy
        self.memcpy_func.argtypes = (c_void_p, c_void_p, c_size_t)
        self.memcpy_func.restype = LPVOID
 
        self.rtl_copy_memory_func = self.kernel32_dll.RtlCopyMemory
        self.rtl_copy_memory_func.argtypes = (LPVOID, LPCVOID, c_ulonglong)
 
        self.unmap_view_of_file_func = self.kernel32_dll.UnmapViewOfFile
        self.unmap_view_of_file_func.argtypes = (LPCVOID, )
        self.unmap_view_of_file_func.restype = BOOL
 
        self.memcpy_func = self.msvcrt_dll.memcpy
        self.memcpy_func.argtypes = (c_void_p, c_void_p, c_size_t)
        self.memcpy_func.restype = LPVOID
 
        self.rtl_copy_memory_func = self.kernel32_dll.RtlCopyMemory
        self.rtl_copy_memory_func.argtypes = (LPVOID, LPCVOID, c_ulonglong)
 
        self.unmap_view_of_file_func = self.kernel32_dll.UnmapViewOfFile
        self.unmap_view_of_file_func.argtypes = (LPCVOID, )
        self.unmap_view_of_file_func.restype = BOOL
 
        self.close_handle_func = self.kernel32_dll.CloseHandle
        self.close_handle_func.argtypes = (HANDLE, )
        self.close_handle_func.restype = BOOL
 
        self.get_last_error_func = self.kernel32_dll.GetLastError
        self.getch_func = self.msvcrt_dll._getch
        ''' C타입  함수 포인터들의 선언 '''    
    
    
        self.file_mapping_name_ptr = c_wchar_p(nameOfMmf)
        
 
        self.mapping_handle = self.create_file_mapping_func(self.INVALID_HANDLE_VALUE, 0, self.PAGE_READWRITE, 0, self.SHMEMSIZE, self.file_mapping_name_ptr)
    
        print("Mapping object handle: 0x{:016X}".format(self.mapping_handle))
        if not self.mapping_handle:
            print("Could not open file mapping object: {:d}".format(self.get_last_error_func()))
            return 1
            raise WinError()
    
    def update(self, data) :
        
        if(type(data) != type(b"")):
            print("data의 타입이 바이트 타입이 아닙니다.")
            return 1
        
        self.data = bytes(data)
        self.msg_ptr = c_char_p(self.data)
        self.mapped_view_ptr = self.map_view_of_file_func(self.mapping_handle, self.FILE_MAP_ALL_ACCESS, 0, 0, self.SHMEMSIZE)
    
    
        print("Mapped view addr: 0x{:016X}".format(self.mapped_view_ptr))
    
        if not self.mapped_view_ptr:
            print("Could not map view of file: {:d}".format(self.get_last_error_func()))
            self.close_handle_func(self.mapping_handle)
            return 1
            raise WinError()        
 
        print("Message length: {:d} chars ({:d} bytes)".format(len(self.data), len(self.data)))
   
    
        self.memcpy_func(self.mapped_view_ptr, self.msg_ptr, len(self.data))    
        
        
        
    def read(self) :
        self.read_handle = self.open_file_mapping_func(self.FILE_MAP_READ, 0, self.file_mapping_name_ptr)
    
        if not self.read_handle:
            print("Could not open file mapping object: {:d}".format(self.get_last_error_func()))
            return 1
            raise WinError()
            
        
        self.readMapped_view_ptr = self.map_view_of_file_func(self.read_handle, self.FILE_MAP_READ, 0, 0, self.SHMEMSIZE)
    
        if not self.readMapped_view_ptr:
            print("Could not map view of file: {:d}".format(self.get_last_error_func()))
            self.close_handle_func(self.mapping_handle)
            return 1
            raise WinError()   
        
        
        DATA = c_char_p(self.readMapped_view_ptr)
        
        return DATA.value
 
 
    def close(self) :
        self.unmap_view_of_file_func(self.mapped_view_ptr)
        self.close_handle_func(self.mapping_handle)
 
 
 
 
 
if __name__ == "__main__":
    print("Python {:s} on {:s}".format(sys.version, sys.platform)) 
    #rtl_copy_memory_func(mapped_view_ptr, msg_ptr, byte_len)
    
    print("Hit a key to read...")
    input()
    
    mmf = Mmf("123123")   
    mmf.read();
    
    
    print("Hit a key to exit...")
    input()
    mmf.close()
     
