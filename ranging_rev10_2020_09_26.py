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

	def start_ranging(self, print_counter):
		device_range = DeviceRange()
		status = self.pozyx.doRanging(
			self.destination_id, device_range, self.remote_id)
		if status == POZYX_SUCCESS:
			if print_counter==0 : print("* " + str(self.name) + " Ranging Success :" + str(device_range))
			return(device_range)
		else:
			# print(str(self.name) + ' has failed to ranging.')
			self.setup()
			# print('Setup again.')
			# print('Restarting "start_ranging" method.')
			new_device_range = self.start_ranging(print_counter)
			return(new_device_range)

 
 
if __name__ == "__main__":

	##########################################################
	### Initializing "Socket Communication" with TCP
	##########################################################

	# TCP_IP = '192.168.10.251'

	TCP_IP = '192.168.0.5'
	TCP_PORT = 10801 # C# 송신단 서버의 IP 및 포트 번호 

	s = socket(AF_INET, SOCK_STREAM)
	s.connect((TCP_IP, TCP_PORT))


	##########################################################
	### Initializing "POZYX" ranging & positioning
	##########################################################

	remote_id = None



	##  3   4
	##
	##  1   2
	destination_id = [0x672c, 0x677d, 0x6739, 0x6704 ]
	ranging_protocol = PozyxConstants.RANGE_PROTOCOL_PRECISION
	# ranging_protocol = PozyxConstants.RANGE_PROTOCOL_FAST

	pozyx = PozyxSerial(get_first_pozyx_serial_port())
	#pozyx = PozyxSerial('COM15')

	start_time = time.time()
	loop_idx = 0

	#폴
	H = np.array([ [5.4, 0], [0,8.4], [5.4,8.4]])#*1000 # mm unit.
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

	print_counter=0
	num_loop_for_print=10


while (1):
	print_counter=print_counter%num_loop_for_print

	# print("print_counter is : ", print_counter)
	if(print_counter==0):
		print('\n\n\n-------------------------------------------------------------------------\n')
		print('* This messaage is printed once every',num_loop_for_print,'times.\n')

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

	range0 = r0.start_ranging(print_counter)
	# print(range0)
	# print('range0 is done. \n')
	range1 = r1.start_ranging(print_counter)
	# print(range1)
	# print('range1 is done. \n')
	range2 = r2.start_ranging(print_counter)
	# print(range2)
	# print('range2 is done. \n')
	range3 = r3.start_ranging(print_counter)
	# print(range3)
	# print('range3 is done. \n')
	# range4 = r4.start_ranging()
	# print(range4)
	# print('range4 is done. \n')

	# # known_anchors = np.array([ [   0,  0, 200], [ 761, 1464, 200], [ -25, 1518, 200], [ 713,  755, 200] ])
	# known_pos = np.array([ 240-7,  600+21, 110])
	# known_ranges=np.round(np.sqrt(np.diag(np.matmul((known_anchors-known_pos), np.transpose(known_anchors-known_pos))))*10)

	# measured = np.array([range0.distance, range1.distance, range2.distance, range3.distance])
	# print('real range is : \n')
	# print('difference : measured - known : ', measured-known_ranges)

	ranges_without_0 = np.array([range1.distance,range2.distance,range3.distance])/1000
	range_0_square = np.square((range0.distance /1000)) 
	range_square_s = ranges_without_0 * ranges_without_0
	b = (K_square - range_square_s + range_0_square)/2

	position = np.matmul(H_,b)
	# POZYX 모듈과 안테나 사이의 실제거리 보정 (0.07,-0.21) + AP2 중심 -> Tx 중심 좌표계로 이동 (-2.40,+3.60)
	# position = position + np.array([-2.33, +3.39]) 

	time_until_now = time.time() - start_time
	loop_idx = loop_idx+1
	ave_proc_time_per_loop = time_until_now/loop_idx
	if(print_counter==0):
		print('Here is : ', (position), "(m)\n")#/1000+np.array([0,0,1.6])
	# print('average process time per loop is : ', ave_proc_time_per_loop * 1000, 'ms.\n')
	# print('Acheievable ranging rate is : ', 1/ave_proc_time_per_loop, 'times/sec. \n\n\n')
	
	MsgPos = []

	MsgPosX = str(round(position[0],8))
	MsgPosY = str(round(position[1] -0.25,8))

	# print('len of MsgPos[0] : ', len(MsgPosX))
	# print('len of MsgPos[1] : ', len(MsgPosY))

	if(len(MsgPosX) < 10): # 1의 자리와 소수점 때문에 두 개 증가
		Diff = 10 - len(MsgPosX)
		for _ in range(Diff):
			MsgPosX = MsgPosX + '0'

	if(len(MsgPosY) < 10): # 1의 자리와 소수점 때문에 두 개 증가
		Diff = 10 - len(MsgPosY)
		for _ in range(Diff): 
			MsgPosY = MsgPosY + '0'

	# print('revised len of MsgPos[0] : ', len(MsgPosX))
	# print('revised len of MsgPos[1] : ', len(MsgPosY))

	Message = str(position)+ '\n'
	Message = '['+MsgPosX + ' ' + MsgPosY + str(0)+']' + '\n'
	Message = '['+MsgPosX + ' ' + MsgPosY + str(0)+']'
	mmfMsg = '['+MsgPosX + ' ' + MsgPosY +']'
	mmf_RxPos = MMF_write.Mmf("RxPos", str(mmfMsg).encode('utf-8'))
	mmf_RxPos.update()
	MMF_write.Mmf("RxPos", str(mmfMsg).encode('utf-8')).update()   
	pos_read_back = mmf_RxPos_read.read() ##### 이걸 flush를 어떻게 해 주지???

	s.send(Message.encode('utf-8'))
	if(print_counter==0):
		print('so the MsgPosX and MsgPosY are : ', MsgPosX, 'and', MsgPosY)
		print("MMF_RxPos is : ", pos_read_back)
		# print("my return is : ", position[0], position[1])
		print('Following message: is sent : ', Message)

		print('\n-------------------------------------------------------------------------')
	time.sleep(0.1)

	### 하나라도 'None'이면 루프를 중단.
	### 근데 위에서 'None'인 경우가 없게끔 보장해 주었으므로, 코드가 끝날 일은 없다.
	if str(range0) == 'None' or str(range1) == 'None' or str(range2) == 'None' or str(range3) == 'None':
		break

	print_counter=print_counter+1
		
