from pypozyx import (PozyxSerial, PozyxConstants, version,
					 SingleRegister, DeviceRange, POZYX_SUCCESS, POZYX_FAILURE, get_first_pozyx_serial_port)

from pypozyx.tools.version_check import perform_latest_version_check
from socket import *

import time
import numpy as np
import MMF_write
import MMF_read

class ReadyToRange(object):
	def __init__(self, pozyx, destination_id, protocol=PozyxConstants.RANGE_PROTOCOL_PRECISION,
				 remote_id=None):
		self.pozyx = pozyx
		self.destination_id = destination_id
		self.remote_id = remote_id
		self.protocol = protocol

	def setup(self):
		self.pozyx.setRangingProtocol(self.protocol, self.remote_id)

	def start_ranging(self):
		device_range = DeviceRange()
		status = self.pozyx.doRanging(
			self.destination_id, device_range, self.remote_id)
		if status == POZYX_SUCCESS:
			print("No problem. return the data.")
			return(device_range)
		else:
			print('There is a problem. Something is wrong in received data..')
			self.setup()
			print('re-setup complete..')
			new_device_range = self.start_ranging()
			print('"start_ranging" method re-started.')
			return(new_device_range)

 
 
if __name__ == "__main__":

	##########################################################
	### Initializing "Socket Communication" with TCP
	##########################################################

	TCP_IP = '192.168.0.221'
	TCP_PORT = 10801

	s = socket(AF_INET, SOCK_STREAM)
	s.connect((TCP_IP, TCP_PORT))


	##########################################################
	### Initializing "POZYX" ranging & positioning
	##########################################################

	remote_id = None
	destination_id = [0x6932, 0x673f, 0x672d, 0x675a ]
	ranging_protocol = PozyxConstants.RANGE_PROTOCOL_PRECISION
	# ranging_protocol = PozyxConstants.RANGE_PROTOCOL_FAST

	pozyx = PozyxSerial(get_first_pozyx_serial_port())
	#pozyx = PozyxSerial('COM15')

	start_time = time.time()
	loop_idx = 0

	#Constants here!!
	#pos0 = []
	H = np.array([[6,0], [0,9], [6,9]])#*1000 # mm unit.
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
	range4 = 'None'

	# while str(range0) == 'None' or str(range1) == 'None' or str(range2) == 'None' or str(range3) == 'None':
	r0 = ReadyToRange(pozyx, destination_id[0], ranging_protocol, remote_id)    
	r1 = ReadyToRange(pozyx, destination_id[1], ranging_protocol, remote_id)
	r2 = ReadyToRange(pozyx, destination_id[2], ranging_protocol, remote_id)
	r3 = ReadyToRange(pozyx, destination_id[3], ranging_protocol, remote_id)
	# r4 = ReadyToRange(pozyx, destination_id[4], ranging_protocol, remote_id)

	r0.setup()
	r1.setup()
	r2.setup()
	r3.setup()
	# r4.setup()

	range0 = r0.start_ranging()
	print(range0)
	print('range0 is done. \n')
	range1 = r1.start_ranging()
	print(range1)
	print('range1 is done. \n')
	range2 = r2.start_ranging()
	print(range2)
	print('range2 is done. \n')
	range3 = r3.start_ranging()
	print(range3)
	print('range3 is done. \n')
	# range4 = r4.start_ranging()
	# print(range4)
	# print('range4 is done. \n')

	ranges_without_0 = np.array([range1.distance,range2.distance,range3.distance])/1000
	range_0_square = np.square((range0.distance /1000)) # 여기서 높이 보정
	range_square_s = ranges_without_0 * ranges_without_0 # 여기서 높이 보정
	b = (K_square - range_square_s + range_0_square)/2

	position = np.matmul(H_,b)

	time_until_now = time.time() - start_time
	loop_idx = loop_idx+1
	ave_proc_time_per_loop = time_until_now/loop_idx
	print('Here is : ', (position), "(m)")#/1000+np.array([0,0,1.6])
	print('average process time per loop is : ', ave_proc_time_per_loop * 1000, 'ms.\n')
	print('The number of ranging per time is : ', 1/ave_proc_time_per_loop, 'times/sec. \n\n\n')

	position = position + np.array([0.07, -0.21])

	MsgPos = []

	MsgPosX = str(round(position[0],8))
	MsgPosY = str(round(position[1],8))

	print('len of MsgPos[0] : ', len(MsgPosX))
	print('len of MsgPos[1] : ', len(MsgPosY))

	if(len(MsgPosX) < 10): # 1의 자리와 소수점 때문에 두 개 증가
		Diff = 10 - len(MsgPosX)
		for _ in range(Diff):
			MsgPosX = MsgPosX + '0'

	if(len(MsgPosY) < 8+2): # 1의 자리와 소수점 때문에 두 개 증가
		Diff = 10 - len(MsgPosY)
		for _ in range(Diff): 
			MsgPosY = MsgPosY + '0'

	print('revised len of MsgPos[0] : ', len(MsgPosX))
	print('revised len of MsgPos[1] : ', len(MsgPosY))

	# Message = str(position)+ '\n'
	Message = '['+MsgPosX + ' ' + MsgPosY + ']' + '\n'
	mmf_RxPos = MMF_write.Mmf("RxPos", str(position).encode('utf-8'))   
	mmf_RxPos.update()
	pos_read_back = mmf_RxPos_read.read() ##### 이걸 flush를 어떻게 해 주지???
	print("MMF_RxPos is : ", pos_read_back)


	s.send(Message.encode('utf-8'))
	# print("my return is : ", position[0], position[1])
	print('Following message: is sent : ', Message)
	time.sleep(0.5)

	### 하나라도 'None'이면 루프를 중단.
	### 근데 위에서 'None'인 경우가 없게끔 보장해 주었으므로, 코드가 끝날 일은 없다.
	if str(range0) == 'None' or str(range1) == 'None' or str(range2) == 'None' or str(range3) == 'None':
		break

		
