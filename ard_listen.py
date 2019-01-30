import serial
import os
import time
import subprocess
import sys, select
import numpy as np
import csv
import os.path
#from scipy.signal import savgol_filter
import peakutils

#print "ard_listen version 20151022"
#os.system("python bad_script 2> error_log.txt")
#print "sleeping for 30 seconds ..."
#time.sleep(30)
#print "continuing script ..."
ser = serial.Serial ('/dev/ttyACM0', 9600)

def main():
	print "ard_listen version 20161019"
	#os.system("python bad_script 2> error_log.txt")
	#print "sleeping for 30 seconds ..."
	#time.sleep(30)
	#print "continuing script ..."
	ser = serial.Serial ('/dev/ttyACM0', 9600)
	while 1:
		txt = ser.readline()
		print txt
		if txt[:3] == "zz:":
			#print txt[:3]
			print "pi found PID data! File no: " + ard_fileno
			datafile.write(txt[:-2] + "\r\n")	
		if txt[:3] == "aa:":
			#print txt[:3]		
			print "pi found current file index!"		
			print txt[4:]
			#filename = "".join(["/media/PI_CRUZ2/data/", txt[4:-2], ".CSV"])
			filename = "".join([year_month_folder, txt[4:-2], ".CSV"])
			ard_fileno = txt[4:-2] # needed for chrom_file
			print filename
			datafile = open(filename, "w", 1)
			datafile.write(txt[:-2] + "\r\n")	
		if txt[:3] == "ab:":
			#print txt[:3]
			print "pi found date_time stamp!"
			datafile.write(txt[:-2] + "\r\n")
		if txt[:3] == "ac:":
			#print txt[:3]
			print "pi found sample type!"
			datafile.write(txt[:-2] + "\r\n")
		if txt[:3] == "ad:":
			#print txt[:3]
			print "pi found column temperature!"
			datafile.write(txt[:-2] + "\r\n")
		if txt[:3] == "ae:":
			#print txt[:3]
			print "pi found arduino reset event!"
		if txt[:3] == "af:":
			#print txt[:3]
			print "pi found arduino pause event!"
		if txt[:3] == "ag:":
			#print txt[:3]
			print "pi found arduino resume event!"
		if txt[:3] == "ah:":
			#print txt[:3]
			print "pi found pi reboot event!"
			os.system("sudo reboot")
			time.sleep(10) #10 s delay
		if txt[:3] == "ai:":
			#print txt[:3]
			print "pi found pi shutdown event!"
			os.system("sudo shutdown -h now")
			time.sleep(10) #10 s delay
		if txt[:3] == "aj:":
			#print txt[:3]
			print "pi found column power on event!"
		if txt[:3] == "ak:":
			#print txt[:3]
			print "pi found column power off event!"
		if txt[:3] == "al:":
			#print txt[:3]
			print "pi found PID power on event!"
		if txt[:3] == "am:":
			#print txt[:3]
			print "pi found PID power off event!"	
		if txt[:3] == "an:":
			#print txt[:3]
			print "pi found chrom file name new OK!"
			datafile.write(txt[:-2] + "\r\n")
		if txt[:3] == "ao:":
			#print txt[:3]
			print "pi found Chrom file already existed KO!"	
			datafile.write(txt[:-2] + "\r\n")
		if txt[:3] == "ap:":
			#print txt[:3]
			print "pi found max number of chrom cycles reached"	
		if txt[:3] == "aq:":
			#print txt[:3]
			print "pi found altimeter pressure"
			datafile.write(txt[:-2] + "\r\n")
		if txt[:3] == "ar:":
			#print txt[:3]
			print "pi found altimeter temperature"
			datafile.write(txt[:-2] + "\r\n")
		if txt[:3] == "as:":
			#print txt[:3]
			print "pi found integrated volume"
			datafile.write(txt[:-2] + "\r\n")
		if txt[:3] == "at:":
			#print txt[:3]
			print "pi found average flow rate"
			datafile.write(txt[:-2] + "\r\n")
		if txt[:3] == "au:":
			#print txt[:3]
			print "pi found volume integration time"
			datafile.write(txt[:-2] + "\r\n")	
		if txt[:3] == "av:":
			#print txt[:3]
			print "pi found chromatogram done"	
			print "closing pi datafile"
			print filename
			datafile.close()
			print "now making quick plot ..."
			print "reading arduino chrom file"
			#ard_filename = "".join(["/media/PI_CRUZ2/data/", ard_fileno, ".CSV"])
			ard_filename = "".join([year_month_folder, ard_fileno, ".CSV"])
			print ard_filename
			ard_datafile = open(ard_filename, "r", 1)
			chrom_filename = "".join(["/media/PI_CRUZ2/data/", "chrom_file.CSV"])
			print chrom_filename
			chrom_datafile = open(chrom_filename, "w", 1)
			time_inc = 0
			chrom_datafile.write("fileno: " + ard_fileno + "\r\n")
			while 1:
				line = ard_datafile.readline()
				if not line: break
				#print"Read line %s" % (line)
				if line[:3] == "zz:":
					#chrom_datafile.write(str(time_inc))
					chrom_datafile.write((str(time_inc) + " " + line[4:]))
					time_inc+=0.2
			ard_datafile.close()
			chrom_datafile.close()
			#obph(year_month_folder, ard_filename)
			obph(year_month_folder, filename)
			ard_datafile.close()
			# Now copy raw file to dropbox
			#dbox_filename = "".join(["/data/", ard_fileno, ".CSV"])
			#dbox_cmd = "lxterminal -e /home/pi/Dropbox-Uploader/dropbox_uploader.sh upload {0} {1}".format(ard_filename,dbox_filename)
			#os.system(dbox_cmd)
			# Now call gnuplot and communicate the chrom_file.csv ...
			plot = subprocess.Popen(['gnuplot'], stdin=subprocess.PIPE)
			plot.communicate("""
			set title "Quick look chrom"
			set xlabel "Time (s)"
			set ylabel "PID signal counts"
			plot "/media/PI_CRUZ2/data/chrom_file.CSV" using 1:2 title column with line
			pause 10
			""")
		if txt[:3] == "aw:":
			#print txt[:3]
			print "pi found main column flow on event!"
		if txt[:3] == "ax:":
			#print txt[:3]
			print "pi found main column flow off event!"	
		if txt[:3] == "ay:":
			#print txt[:3]
			print "pi found pre-column backflush flow on event!"	
		if txt[:3] == "az:":
			#print txt[:3]
			print "pi found pre-column backflush flow off event!"	
		if txt[:3] == "ba:":
			#print txt[:3]
			print "pi found trap heating on event!"
		if txt[:3] == "bb:":
			#print txt[:3]
			print "pi found trap heating off event!"
		if txt[:3] == "bc:":
			#print txt[:3]
			print "pi found trap cold temperature!"
			datafile.write(txt[:-2] + "\r\n")	
		if txt[:3] == "bd:":
			#print txt[:3]
			print "pi found trap hot temperature!"
			datafile.write(txt[:-2] + "\r\n")	
		if txt[:3] == "be:":
			#print txt[:3]
			print "pi found valco valve switched to inject!"
		if txt[:3] == "bf:":
			#print txt[:3]
			print "pi found valco valve switched to load!"
		if txt[:3] == "bg:":
			#print txt[:3]
			print "pi found sample pump pressure differential!"
			datafile.write(txt[:-2] + "\r\n")	
		if txt[:3] == "bh:":
			#print txt[:3]
			print "pi found starting config edit mode"
			start_config()					
		if txt[:3] == "bi:":
			#print txt[:3]
			print "pi found ending config edit mode"
		if txt[:3] == "bj:":
			#print txt[:3]
			print "pi found enter max no of chroms to run (1-999, 999 = forever)"			
			range_check(0, 1000)
		if txt[:3] == "bk:":
			#print txt[:3]
			print "pi found enter max trap load time, seconds (1-600)"
			range_check(0, 601)
		if txt[:3] == "bl:":
			#print txt[:3]
			print "pi found enter inject time, seconds (30-300)"
			range_check(29, 301)	
		if txt[:3] == "bm:":
			#print txt[:3]
			print "pi found enter backflush time, seconds (30-300)"
			range_check(29, 301)
		if txt[:3] == "bn:":
			#print txt[:3]
			print "pi found enter target sample volume, scc (2-200)"
			range_check(1, 201)
		if txt[:3] == "bo:":
			#print txt[:3]
			print "pi found enter volume integration timeout, seconds (60-600)"
			range_check(59, 601)	
		if txt[:3] == "bp:":
			#print txt[:3]
			print "pi found enter target pump pressure differential, kPa (5-25)"
			range_check(4, 26)		
		if txt[:3] == "bq:":
			#print txt[:3]
			print "pi found enter number of samples to run between cals (0-999)"
			range_check(-1, 1000)													
		if txt[:3] == "br:":
			#print txt[:3]
			print "pi found enter target calibration volume, scc (2-200)"
			range_check(1, 201)	
		if txt[:3] == "ck:":
			#print txt[:3]
			print "pi found enter dual sampling mode, 0 = sam1 only (0-1)"
			range_check(-1, 2)	
		if txt[:3] == "bs:":
			#print txt[:3]
			print "pi found sample1 valve on (pneumax 1 on)!"	
		if txt[:3] == "bt:":
			#print txt[:3]
			print "pi found sample1 valve off (pneumax 1 off)!"	
		if txt[:3] == "bu:":
			#print txt[:3]
			print "pi found sample2 valve on (pneumax 2 on)!"	
		if txt[:3] == "bv:":
			#print txt[:3]
			print "pi found sample2 valve off (pneumax 2 off)!"	
		if txt[:3] == "bw:":
			#print txt[:3]
			print "pi found cal valve on (pneumax 3 on)!"	
		if txt[:3] == "bx:":
			#print txt[:3]
			print "pi found cal valve off (pneumax 3 off)!"	
		if txt[:3] == "by:":
			#print txt[:3]
			print "pi found blank valve on (pneumax 4 on)!"	
		if txt[:3] == "bz:":
			#print txt[:3]
			print "pi found blank valve off (pneumax 1 off)!"
		if txt[:3] == "ca:":
			#print txt[:3]
			print "pi found request for R-Pi system temperature!"	
			sys_T = get_sysTemp()
			print("R-Pi system temp = ", sys_T)
			datafile.write(txt[:4] + sys_T + "\r\n")	
		if txt[:3] == "cb:":
			#print txt[:3]
			print "pi found input DC voltage!"
			datafile.write(txt[:-2] + "\r\n")	
		if txt[:3] == "co:":
			#print txt[:3]
			print "pi found starting setup mode"
			setup_mode()				
		if txt[:3] == "cc:":
			#print txt[:3]
			print "pi found starting datetime edit mode"
			start_datetime_edit()					
		if txt[:3] == "cd:":
			#print txt[:3]
			print "pi found ending datetime edit mode"
		if txt[:3] == "ce:":
			#print txt[:3]
			print "pi found enter year in form yy (0-99)"
			range_check(-1, 100)
		if txt[:3] == "cf:":
			#print txt[:3]
			print "pi found enter month in form mm (1-12)"
			range_check(0, 13)
		if txt[:3] == "cg:":
			#print txt[:3]
			print "pi found enter day in form dd (1-31)"
			range_check(-1, 32)
		if txt[:3] == "ch:":
			#print txt[:3]
			print "pi found enter hour in form hh (0-23)"
			range_check(-1, 24)
		if txt[:3] == "ci:":
			#print txt[:3]
			print "pi found enter minute in form mm (0-59)"
			range_check(-1, 60)
		if txt[:3] == "cj:":
			#print txt[:3]
			print "pi found enter seconds in form ss (0-59)"
			range_check(-1, 60)	
		if txt[:3] == "cl:":
			#print txt[:3]
			print "pi found starting manual control mode"
			start_manual_control()	
		if txt[:3] == "cp:":
			#print txt[:3]
			print "pi found ending setup mode"			
		#if txt[:3] == "cm:":
			#print txt[:3]
			#print "pi found ending manual control mode"
		if txt[:3] == "cq:":
			#print txt[:3]
			print "pi found year and month folder"	
			year_month_folder = "".join(["/media/PI_CRUZ2/data", txt[4:-2]])	
			print year_month_folder	
			if not os.path.exists(year_month_folder):																											
				os.makedirs(year_month_folder)
				print "new data folder"
			else:
				print "data folder exists"
				
def obph(path_name, chrom_name):
    # creates a list of counts from the data file that can be plotted
    with open(chrom_name, 'rb') as curve:
        data = csv.reader(curve)
        vals = []    
        for row in data:
            mstr = str(row[0])
            mstr1 = str(row[1])       
            str1= ','.join(row)
            vals.append(mstr1)
            vals2 = vals[15:] 
    
    #creates a list of time values from the length of the previously created list divided by 5
    lennum = list((range(len(vals2))))
    time = []
    for times in lennum:
        r = times/5
        time.append(r)
	
    vals3 = [float(i) for i in vals2] #converts these values to floating point numbers
    
    #plots the chromatogram
    y = vals3
    x = time
    x = np.array(x)
    y = np.array(y)
    indexes = peakutils.indexes(y, thres=0.01, min_dist=20)
    cent = peakutils.peak.centroid(x, y)

    #y = savgol_filter(y, 19, 2) #runs a smoothing algorithm over the data

    y2 = y+np.polyval([0.002, -0.009, 5], x) #calculates a basline

    base = peakutils.baseline(y2, 2) #removes a baseline from the plot

    indexes = peakutils.indexes((y2-base), thres=0.01, min_dist=20)
    k = 0
    rettam = []
    rettom = []
    for nums in x[indexes]:
        rettam.append(nums)
        pkht = []    
        for muns in (y2-base)[indexes]:
            pkht.append(muns)
    for glums in rettam:
		rettom.append(glums/60)
    
    #Creates a list where  the first term is a value for retention time and the second value is peak height between a 
    #time where the retention time of isoprene is known to fall
    rettim = [float(i) for i in rettom] #converts these values to floating point numbers
    isopk1 = []
    for nums in rettim:
        if rettim[k] > 0.75 and rettim[k] <1.4:
            print "Peak retention time:", rettim[k], ' and peak height is: %.3f' % pkht[k]
            isopk1.append(vals[0])
            isopk1.append(rettim[k])
            isopk1.append(pkht[k])
        k += 1
        
    #This bloc defines variables fileno and the file path and also coverts isopk to an array
    isopk2 = [float(i) for i in isopk1]
    isopk = ['%.3f' % elem for elem in isopk2]
    fileno = 0
    pkht_filename = "".join([path_name, "peak_heights.csv"])
    isopk_ar = np.asarray(isopk)
    isopk_a =  np.reshape(isopk_ar, ((len(isopk_ar)/3), 3))
    
    #This bloc checks if peak_heights file exists and if not, writes the headers
    headers = ["File No.", "Retention Time / s", "Peak Height / counts"]
    if os.path.exists(pkht_filename) == False:
        with open(pkht_filename, "wb") as fp:
            wr = csv.writer(fp)        
            wr.writerow(headers)
    else:
		print "file already exists"
	
	#This bloc defines a variable fileno that corresponds to the last chrom number
    with open(pkht_filename, "a") as fp:
                with open(pkht_filename) as ft:
					my_file = []
					treader = csv.reader(ft, delimiter=' ', quotechar='|')
					for row in treader:
						my_file.append(', '.join(row))                 
    print "hello everyone, what a nice day"
    if my_file:
        row = my_file[0]
        fileno = row[0:((len(isopk[0])))]
    else:
        print "my_file empty"
        fileno = -1
	
    #This bloc checks if the record already exists, and if not, writes the new data to the file 
    if isopk[0] == fileno:
		print "This file has already been added to the list of results"
    else:
        with open(pkht_filename, "a") as fp:
            with open(pkht_filename, "r") as ft:
            
                wr = csv.writer(fp)    
                np.savetxt(fp, isopk_a, delimiter=",", fmt='%s') 
     	
	
def setup_mode():
	print " Type 's' then enter for setup menu (60 sec time-out, 'exit' anytime to skip)"
	done = 0
	print(" >> "),
	sys.stdout.flush()
	while(done == 0):
		#get keyboard input
		i, o, e = select.select( [sys.stdin], [], [], 60 )
		if (i):
			txt = sys.stdin.readline().strip()
			#print "You entered: ", txt
			if txt == "exit":
				if done == 0:
					print "skipping setup"
				#else:
					#print "using existing values for remaining datetime fields"
				print " waiting for 60 second time-out ..."
				done = 1
				ser.write('q')
			if txt == "s":
				#print "sending s"
				ser.write(txt)	
				done = 1
			else:
				if done == 0:
					print "Type 's' then enter to enter setup menu (60 sec time-out, 'exit' anytime to skip)"
					print(" >> "),
					sys.stdout.flush()
		else:
			print "Time-out!"
			done = 1		
	
					
def start_config():
	print " Type 'c' then enter to edit configs (60 sec time-out, 'exit' anytime to skip)"
	done = 0
	print(" >> "),
	sys.stdout.flush()
	while(done == 0):
		#get keyboard input
		i, o, e = select.select( [sys.stdin], [], [], 60 )
		if (i):
			txt = sys.stdin.readline().strip()
			#print "You entered: ", txt
			if txt == "exit":
				if done == 0:
					print "using current config values"
				else:
					print "using default values for remaining configs"
				print " waiting for 60 second time-out ..."
				done = 1
				ser.write('q')
			if txt == "c":
				#print "sending c"
				ser.write(txt)	
				done = 1
			else:
				if done == 0:
					print "Type 'c' then enter to edit configs (60 sec time-out, 'exit' anytime to skip)"
					print(" >> "),
					sys.stdout.flush()
		else:
			print "Time-out!"
			done = 1						

	
def start_datetime_edit():
	print " Type 't' then enter to edit datetime (60 sec time-out, 'exit' anytime to skip)"
	done = 0
	print(" >> "),
	sys.stdout.flush()
	while(done == 0):
		#get keyboard input
		i, o, e = select.select( [sys.stdin], [], [], 60 )
		if (i):
			txt = sys.stdin.readline().strip()
			#print "You entered: ", txt
			if txt == "exit":
				if done == 0:
					print "using current datetime values"
				else:
					print "using existing values for remaining datetime fields"
				print " waiting for 60 second time-out ..."
				done = 1
				ser.write('q')
			if txt == "t":
				#print "sending t"
				ser.write(txt)	
				done = 1
			else:
				if done == 0:
					print "Type 't' then enter to edit datetime (30 sec time-out, 'exit' anytime to skip)"
					print(" >> "),
					sys.stdout.flush()
		else:
			print "Time-out!"
			done = 1		
	
	
def start_manual_control():
	print " Type 'm' then enter for manual control mode (60 sec time-out, 'exit' to skip)"
	done = 0
	change = 1
	print(" >> "),
	sys.stdout.flush()
	while(done == 0):
		#get keyboard input
		i, o, e = select.select( [sys.stdin], [], [], 1 ) # use short 1 sec timeout as overal timeout controlled by arduino
		if (i):
			txt = sys.stdin.readline().strip()
			#print "You entered: ", txt
			if txt == "exit":
				if done == 0:
					print "continuing ..."
				else:
					print "exiting manual control ..."
				done = 1
				ser.write('q')
			if txt == "m":
				#print "sending m"
				ser.write(txt)	
				txt = ser.readline()
				print txt
				done = 2
			else:
				if done == 0:
					print "Type 'm' then enter for manual control mode (60 sec time-out, 'exit' to skip)"
					print(" >> "),
					sys.stdout.flush()
		else:
			#print "Time-out!"
			done = 0
		txt = ser.inWaiting()
		if(txt):	
			#print "txt: " + str(txt)
			txt = ser.readline()
			print "txt: " + str(txt)
			if txt[:3] == "cl:":
				print txt[:3]
				print "pi found ending manual control mode"
				done = 1
	if(done == 2):
		done = 0
		while(done == 0):
			if change:
				print "Enter valve number to toggle 1-8 or 'p' for pump (1 hr time-out, 'exit' to skip)"
				print(" >> "),
				sys.stdout.flush()
				change = 0
			#print(" >> "),
			#get keyboard input
			i, o, e = select.select( [sys.stdin], [], [], 1 ) # use short 1 sec timeout as overal timeout controlled by arduino
			if (i):
				txt = sys.stdin.readline().strip()
				#print "You entered: ", txt
				if txt == "exit":
					if done == 0:
						print "exiting toggle mode"
					done = 1
					ser.write('q')
				if (txt >= "1" and txt<= "8"):
				#if ((txt >= "1" and txt<= "8") or txt == "p"):
					ser.write(txt)	
					txt = ser.readline()
					print txt		
					change = 1	
				elif (txt == "p"):	
					ser.write(txt)	
					txt = ser.readline()
					print txt		
					#change = 1	
					while(change == 0):
						txt = ser.inWaiting()
						if(txt):	
							#print "txt: " + str(txt)
							txt = ser.readline()
							print " " + str(txt)						
							if txt[:3] == "cn:":
								#print txt[:3]
								#print "pi found ending pump flowmeter test"
								change = 1	
				else:
					print "invalid input, try harder next time ..."	
			else:
				#print "Time-out!"
				done = 0
				#ser.write('q')
			txt = ser.inWaiting()
			if(txt):	
				#print "txt: " + str(txt)
				txt = ser.readline()
				print "txt: " + str(txt)
				if txt[:3] == "cl:":
					print txt[:3]
					print "pi found ending manual control mode"
					done = 1	


def range_check(lo,hi):
	done = 0
	i = 0
	print(" >> "),
	sys.stdout.flush()
	while(done == 0):
		#get keyboard input
		i, o, e = select.select( [sys.stdin], [], [], 60 )
		if (i):
			txt = sys.stdin.readline().strip()
			#print "You entered: ", txt
			if txt == "exit":
				print "using default values for remaining configs"
				print "waiting for 60 second time-out ..."
				done = 1
			else:
				try:
					int(txt)
				except ValueError:
					print "non-integer, try again ..."
					print(">> "),
					sys.stdout.flush()
				else:
					value = int(txt)
					if lo < value < hi:
						#print "sending " + txt
						ser.write(txt)
						done = 1
					else:
						print "out of range, try again ..."
						print(">> "),
						sys.stdout.flush()
		else:
			print "Time-out!"
			done = 1	
	
	
def get_sysTemp():
	os.system("/opt/vc/bin/vcgencmd measure_temp > /home/pi/adr_python/sysTemp")
	sysfo = open("/home/pi/adr_python/sysTemp") # Open the sysTemp file
	str = sysfo.read() # read characters in sysTemp and hold as 'str'
	sysfo.close() # closed the file
	print(str) # print temp to check
	temp1 = str[5:9] # mine out the required characters
	#temp2 = eval(temp1) # convert string into a number
	#print("system temp = ",temp2) # print value to check
	#temp = temp2*2 # multiply by 2 to check for math suitability
	#print("twice system temp =",temp) # print multiplied number
	return temp1

		
	
	
if __name__ == '__main__':
	main()
