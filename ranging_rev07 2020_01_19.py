from pypozyx import (PozyxSerial, PozyxConstants, version,
					 SingleRegister, DeviceRange, POZYX_SUCCESS, POZYX_FAILURE, get_first_pozyx_serial_port)

from pypozyx.tools.version_check import perform_latest_version_check
from socket import *

import time
import numpy as np
import MMF_write
import MMF_read

class ReadyToRange(object):
	def __init__(self, name, pozyx, destination_id, protocol=PozyxConstants.RANGE_PROTOCOL_PRECISION,
				 remote_id=None):
		self.pozyx = pozyx
		self.destination_id = destination_id
		self.remote_id = remote_id
		self.protocol = protocol
		self.name = name

	def setup(self):
		self.pozyx.setRangingProtocol(self.protocol, self.remote_id)

	def start_ranging(self):
		device_range = DeviceRange()
		status = self.pozyx.doRanging(
			self.destination_id, device_range, self.remote_id)
		if status == POZYX_SUCCESS:
			print(str(self.name) + " Ranging Success.")
			return(device_range)
		else:
			print(str(self.name) + ' has failed to ranging.')
			self.setup()
			print('Setup again.')
			print('Restarting "start_ranging" method.')
			new_device_range = self.start_ranging()
			return(new_device_range)

 
 
if __name__ == "__main__":

	##########################################################
	### Initializing "Socket Communication" with TCP
	##########################################################

	TCP_IP = '192.168.0.221'
	TCP_PORT = 10801

	# s = socket(AF_INET, SOCK_STREAM)
	# s.connect((TCP_IP, TCP_PORT))


	##########################################################
	### Initializing "POZYX" ranging & positioning
	##########################################################

	remote_id = None

	# 실험실 POZYX 세팅에서, 반시계 방향으로 사용. AP NO. 1 6 7 8 12
	# AP 1  : ( 213,    0) -> (   0,    0) 기준으로 전부 좌표계 평행이동하면
	# AP 6  : ( 761, 1524) -> ( 548, 1524)
	# AP 7  : ( -25, 1518) -> (-238, 1518)
	# AP 8  : (   0,  756) -> (-213,  756)
	# AP 12 : ( 713,  755) -> ( 500,  755)



	# AP 8  : (   0,  756) -> (   0,    0)
	# AP 6  : ( 761, 1524) -> ( 761,  768)
	# AP 7  : ( -25, 1518) -> ( -25,  762)
	# AP 12 : ( 713,  755) -> ( 713,   -1)
	# (3,6) : ( 393,  875) -> ( 393,  119)


	destination_id = [0x6e06, 0x6a5e, 0x672d, 0x6a32 ]
	ranging_protocol = PozyxConstants.RANGE_PROTOCOL_PRECISION
	# ranging_protocol = PozyxConstants.RANGE_PROTOCOL_FAST

	pozyx = PozyxSerial(get_first_pozyx_serial_port())
	#pozyx = PozyxSerial('COM15')

	start_time = time.time()
	loop_idx = 0

	#Constants here!!
	#pos0 = []
	H = np.array([ [7.61,7.08,0], [-0.25,7.62,0], [7.13,-0.01,0]])#*1000 # mm unit.
	HTH = np.matmul(np.transpose(H),H)
	HTH_inv = np.linalg.pinv(HTH)
	H_ = np.matmul(HTH_inv, np.transpose(H))
	K_square = np.diag(np.matmul(H,np.transpose(H)))


	##########################################################
	### Memory Mapped File
	##########################################################

	mmf_RxPos = MMF_write.Mmf("RxPos", b"")   
	mmf_RxPos.update()

	mmf_RxPos_read = MMF_read.Mmf("RxPos")


while (1):

	print('\n\n\n\n\n\n\n\n\n\n\n\n\n')
	range0 = 'None'
	range1 = 'None'
	range2 = 'None'
	range3 = 'None'
	# range4 = 'None'

	# while str(range0) == 'None' or str(range1) == 'None' or str(range2) == 'None' or str(range3) == 'None':
	r0 = ReadyToRange('r0', pozyx, destination_id[0], ranging_protocol, remote_id)    
	r1 = ReadyToRange('r1', pozyx, destination_id[1], ranging_protocol, remote_id)
	r2 = ReadyToRange('r2', pozyx, destination_id[2], ranging_protocol, remote_id)
	r3 = ReadyToRange('r3', pozyx, destination_id[3], ranging_protocol, remote_id)
	# r4 = ReadyToRange('r4', pozyx, destination_id[4], ranging_protocol, remote_id)

	r0.setup()
	r1.setup()
	r2.setup()
	r3.setup()
	# r4.setup()

	range0 = r0.start_ranging()
	print(range0)
	# print('range0 is done. \n')
	range1 = r1.start_ranging()
	print(range1)
	# print('range1 is done. \n')
	range2 = r2.start_ranging()
	print(range2)
	# print('range2 is done. \n')
	range3 = r3.start_ranging()
	print(range3)
	# print('range3 is done. \n')
	# range4 = r4.start_ranging()
	# print(range4)
	# print('range4 is done. \n')


	# AP 8  : (   0,  756) -> (   0,    0)
	# AP 6  : ( 761, 1524) -> ( 761,  768)
	# AP 7  : ( -25, 1518) -> ( -25,  762)
	# AP 12 : ( 713,  755) -> ( 713,   -1)
	# (3,6) : ( 393,  875) -> ( 393,  119)
	known_anchors = np.array([ [   0,  756, 200], [ 761, 1464, 200], [ -25, 1518, 200], [ 713,  755, 200] ])
	known_pos = np.array([ 393-7,  875+21, 110])
	known_ranges=np.round(np.sqrt(np.diag(np.matmul((known_anchors-known_pos), np.transpose(known_anchors-known_pos))))*10)

	measured = np.array([range0.distance, range1.distance, range2.distance, range3.distance])

	print('real range is : \n')

	print('difference : measured - known : ', measured-known_ranges)

	ranges_without_0 = np.array([range1.distance,range2.distance,range3.distance])/1000
	range_0_square = np.square((range0.distance /1000)) 
	range_square_s = ranges_without_0 * ranges_without_0
	b = (K_square - range_square_s + range_0_square)/2

	position = np.matmul(H_,b)
	# POZYX 모듈과 안테나 사이의 실제거리 보정 (0.07,-0.21) + AP8 중심 -> 내가쓰던 좌표계로 이동 (-0.93,+4.81)
	position = position + np.array([-0.86, 4.6,0]) 

	time_until_now = time.time() - start_time
	loop_idx = loop_idx+1
	ave_proc_time_per_loop = time_until_now/loop_idx
	print('Here is : ', (position), "(m)\n")#/1000+np.array([0,0,1.6])
	# print('average process time per loop is : ', ave_proc_time_per_loop * 1000, 'ms.\n')
	# print('Acheievable ranging rate is : ', 1/ave_proc_time_per_loop, 'times/sec. \n\n\n')
	
	MsgPos = []

	MsgPosX = str(round(position[0],8))
	MsgPosY = str(round(position[1],8))

	# print('len of MsgPos[0] : ', len(MsgPosX))
	# print('len of MsgPos[1] : ', len(MsgPosY))

	if(len(MsgPosX) < 10): # 1의 자리와 소수점 때문에 두 개 증가
		Diff = 10 - len(MsgPosX)
		for _ in range(Diff):
			MsgPosX = MsgPosX + '0'

	if(len(MsgPosY) < 8+2): # 1의 자리와 소수점 때문에 두 개 증가
		Diff = 10 - len(MsgPosY)
		for _ in range(Diff): 
			MsgPosY = MsgPosY + '0'

	# print('revised len of MsgPos[0] : ', len(MsgPosX))
	# print('revised len of MsgPos[1] : ', len(MsgPosY))

	# Message = str(position)+ '\n'
	Message = '['+MsgPosX + ' ' + MsgPosY + ']' + '\n'
	mmf_RxPos = MMF_write.Mmf("RxPos", str(position).encode('utf-8'))   
	mmf_RxPos.update()
	pos_read_back = mmf_RxPos_read.read() ##### 이걸 flush를 어떻게 해 주지???
	# print("MMF_RxPos is : ", pos_read_back)


	# s.send(Message.encode('utf-8'))
	# print("my return is : ", position[0], position[1])
	print('Following message: is sent : ', Message)
	time.sleep(0.1)

	### 하나라도 'None'이면 루프를 중단.
	### 근데 위에서 'None'인 경우가 없게끔 보장해 주었으므로, 코드가 끝날 일은 없다.
	if str(range0) == 'None' or str(range1) == 'None' or str(range2) == 'None' or str(range3) == 'None':
		break

		
