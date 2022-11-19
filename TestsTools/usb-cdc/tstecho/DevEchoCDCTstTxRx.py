#!/usr/bin/env python

############################### txrxUSB.py ###############################

import serial;
import time;
import argparse
import os
import struct
import sys

__version__ = "20.10.19"

class FatalError(RuntimeError):
	def __init__(self, message):
		RuntimeError.__init__(self, message)

	@staticmethod
	def WithResult(message, result):
		message += " (result was %s)" % hexify(result)
		return FatalError(message)
		
def arg_auto_int(x):
	return int(x, 0)

def main():
	parser = argparse.ArgumentParser(description='USB Echo Utility version %s' % __version__, prog='DevEchoCDCTstTxRx')
	parser.add_argument(
		'--port', '-p',
		help='Serial port device',
		default='COM0')
	parser.add_argument(
		'--count', '-c',
		help='Cycle Count',
		type=arg_auto_int,
		default=1000)
	parser.add_argument(
		'--baud', '-b',
		help='Baud Rate',
		type=arg_auto_int,
		default=3000000)
	parser.add_argument('tx', help='tx count', type=arg_auto_int)
	parser.add_argument('rx', help='rx count', type=arg_auto_int)

	args = parser.parse_args()

	print('USB Echo Utility version %s' % __version__)
	baud = args.baud;
	try:
		serialPort = serial.Serial(args.port, baud, \
								   serial.EIGHTBITS,\
								   serial.PARITY_NONE, \
								   serial.STOPBITS_ONE);
		serialPort.flushInput()
		serialPort.flushOutput()

	except:
		print('Error: Open %s, %d baud!' % (args.port, baud))
		sys.exit(2)

	txcount = args.tx #64*64+4
	rxcount = args.rx #64*64+4
	if txcount > (64*64+4):
		txcount = 64*64+4
	if txcount < 4:
		txcount = 4
	if rxcount > (64*64+4):
		rxcount = 64*64+4
	if rxcount < 4:
		rxcount = 4
	num_Packets = args.count

#--------------------------------

	byteSent = 0;
	byteRead = 0;

	serialPort.timeout = 0.005
	sent = serialPort.write(b'?');
	read = serialPort.read(2048);

	serialPort.timeout = 0.1 + (txcount + rxcount) * 0.0001
	txData = bytearray()
	idx = 0;
	for x in range(txcount-6):
	  txData.append(x&0xff)
	a = 0x55;
	if txcount ==  rxcount:
		b = 0xaa;
	else:
		b = 0x55;
	print('Start', num_Packets, 'cycles', 'Tx', txcount,'-> Rx', rxcount)

	# First time slice
	t1 = time.time()
	for i in range(num_Packets):
		cycle = i+1
		if cycle%20 == 1:
		  print('\rTransaction', cycle, '...', end='')
		  sys.stdout.flush()
		pkt = struct.pack('>BBHH', a, b, rxcount, idx) + txData
		sent = serialPort.write(pkt);
		byteSent += sent;
		read = serialPort.read(rxcount);
		ttt = len(read);
		if ttt == 0:
		  print('\rTransaction', cycle, '- rx timeot')
		  read = serialPort.read(rxcount);
		  ttt = len(read);
		byteRead += ttt
		if ttt != rxcount:
		  t2 = time.time()
		  print('\rTransaction', cycle)
		  print("  Time: %.3f sec" % (t2-t1))
		  print( 'Writes:', byteSent, 'Bytes')
		  print( ' Reads:', byteRead, 'Bytes')
		  print()
		  print('Error: RdBlkLen', ttt, 'Bytes!')
		  sys.exit(1);
		a1, b1, c1, d1 = struct.unpack('>BBHH', read[0:6])
		if a1 != a or b1 != b or txcount != c1 or idx != d1:
		  t2 = time.time()
		  print('\rTransaction', cycle)
		  print("  Time: %.3f sec" % (t2-t1))
		  print('Writes:', byteSent, 'Bytes')
		  print(' Reads:', byteRead, 'Bytes')
		  print()
		  print('Error: DataBlkRd: %02x' % a1, '%02x' % b1, c1, d1, '(!), BlkLen', ttt, 'Bytes')
		  sys.exit(1);
		idx += 1
	# Second time slice
	t2 = time.time()

	throughPut = (byteSent + byteRead)/(t2-t1);

	print('\r----------------------------')
	print("  Time: %.3f sec" % (t2-t1))
	print('Cycles:', i+1, '(transactions)')
	print('Writes:', byteSent, 'Bytes')
	print(' Reads:', byteRead, 'Bytes')
	print('OldBlk: %02x' % a1, '%02x' % b1, '%04x' % c1, '%04x' % d1, '..')
	print('----------------------------')
	if throughPut > 1024:
	  print("IO Speed: %.3f KBytes/s" % (throughPut/1024))
	else:
	  print("IO Speed: %.3f Bytes/s" % (throughPut))
	sys.exit(0)

if __name__ == '__main__':
	try:
		main()
	except FatalError as e:
		print('\nA fatal error occurred: %s' % e)
		sys.exit(2)
