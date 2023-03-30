#!/usr/bin/env python

import serial, string, math, time, calendar, struct


#Add the tf_prefix to the given frame id
def addTFPrefix(frame_id):
    prefix = ""
    prefix_param = rospy.search_param("tf_prefix")
    if prefix_param:
        prefix = rospy.get_param(prefix_param)
        if prefix[0] != "/":
            prefix = "/%s" % prefix

    return "%s/%s" % (prefix, frame_id)

#Novatel Provided Function:
#CRC helping function
def CRC32Value(i):
    CRC = i
    for idx in range(8):
        if (CRC & 1):
            CRC = (CRC >> 1) ^ 0xEDB88320
        else:
            CRC = CRC >> 1
    return CRC

#Novatel Provided Function:
#Calculate the CRC for supplied data (bytearray)
def CalculateBlockCRC32(data):
    CRC = 0;
    for idx in range(len(data)):
        tmp1 = (CRC >> 8) & 0x00FFFFFFFF
        tmp2 = CRC32Value( (CRC^data[idx]) & 0xFF)
        CRC  = tmp1 ^ tmp2
    return CRC


class NovatelParser:
    """
        Parses a binary Novatel Message
    """
    def __init__(self):
        """ Initialize the NovatelParser """
        self.hdr_len     = 0;
        self.hdr_msgID   = 0;
        self.hdr_msgLen  = 0;
        self.hdr_week    = 0;
        self.hdr_msow    = 0;
        self.hdr_rcvStat = 0;

        self.BESTPOS = 42;
        self.BESTVEL = 99;

    def VerifyChecksum(self, data, CRC):
        """ Verify the Checksum, return bool """
        chk = struct.unpack('<L', CRC)
        checksum = CalculateBlockCRC32(bytearray(data))
        return (chk[0] == checksum)
    
    def ParseHeader(self, data):
        """ Read the Header, return bool """
        header = struct.unpack('<BHbBHHBBHlLHH',data);
        self.hdr_len    = header[0]
        self.hdr_msgID  = header[1]
        self.hdr_msgLen = header[4]
        self.hdr_week   = header[8]
        self.hdr_msow   = header[9]
        self.hdr_rcvStat= header[10]
        return (len(data) == self.hdr_len-3)

    def ParseTimeRef(self, timeMsg):
        """ Populate timeMsg from the header info
            timeMsg: ROS message TimeReference
        """
        timeMsg.time_ref = 0; #TODO: Populate the TimeReference message appropriately
        return True
    
    def ParseBestPos(self, data, navMsg, gpsStatus):
        """ Parse a BestPos message, populate navMsg
            data: string
            navMsg: ROS message NavSatFix
            gpsStatus: ROS message GPS Status (gps_common/GPSStatus), holds more information than usual
        """
        bestPos = struct.unpack('<lldddflffffffBBBBBBBB',msg)
        solStat, posType, lat    , lon   , hgt    , und   , datum , \
        latStd , lonStd , hgtStd , stdId , diffAge, solAge, SVs   , \
        slnSVs , obs    , mult   , rsv1  , extStat, rsv2  , sigMsk  = bestPos
        #rospy.loginfo(bestPos)

        # Populate the NavSatFix message
        navMsg.latitude  = lat
        navMsg.longitude = lon
        navMsg.altitude  = hgt
        #TODO: Verify covariance 
        navMsg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        navMsg.position_covariance[0] = latStd
        navMsg.position_covariance[4] = lonStd
        navMsg.position_covariance[8] = hgtStd

        # Populate the Status 
        #TODO: Send a string that represents the status
        navData.status.service = NavSatStatus.SERVICE_GPS
        if solStat == 0 and posType == 50:   # NARROW_INT
            navMsg.status.status = NavSatStatus.STATUS_FIX 
        elif solStat == 0 and posType == 18: # WAAS
            navMsg.status.status = NavSatStatus.STATUS_SBAS_FIX
        else: # catch the rest of the states
            navMsg.status.status = NavSatStatus.STATUS_NO_FIX
            
        # Populate the GPSStatus message
        #  --> Since I dont have enough data to populate every 
        #      element, I'll just do the satellite numbers and status
        gpsStatus.satellites_used = slnSVs
        gpsStatus.satellites_visible = SVs
        gpsStatus.status = solStat
        gpsStatus.position_source = posType
        
        return True

    def ParseBestVel(self, data, velMsg):
        """ Parse a BestPos message, populate velMsg
            data: string
            velMsg: ROS message TwistStamped
        """
        bestVel = struct.unpack('<llffdddf',msg)
        solStat, velType, lat, age, horSpd, trkAngle, vertSpeed, rsv1 = bestVel
        #rospy.loginfo(bestVel)
        #print bestVel

        #TODO: Change the velocity representation if we're going to use it... I dont think this is what we want
        #       I would prefer to send the speed over ground, and then do the math somewhere else
        # Populate the gps's twist message
        velMsg.twist.linear.x = float(horSpd)*0.514444444444*math.sin(math.radians(float(trkAngle)))
        velMsg.twist.linear.y = float(horSpd)*0.514444444444*math.cos(math.radians(float(trkAngle)))

        return True



if __name__ == "__main__":
    GPSLock = False
    count = 0
    each_msg_linelen = 5
    
    posdata = []
    veldata = []
    try:
        #Read in gpsSerial data
        f = open("fwdleft100.gps", 'r')
        all_data = f.readlines()
        
        #print(data)
        sync0 = '\x00'; sync1 = '\x00'; sync2 = '\x00';
        for data in all_data:
        	count = (count + 1) % each_msg_linelen
        	#print(count)
        	if count == 1 and :
        		print(data[0])
        	line_data = data
        	#print(line_data)
        	"""
        	match = '\xAA\x44\x12'
            if sync != match:
                continue

            # READ HEADER
            header      = gpsSerial.read(25)
            if (not parser.ParseHeader(header)):
                print("Packet Failed: Unexpected header size")
                continue

            # READ MESSAGE
            msg = gpsSerial.read(parser.hdr_msgLen)
            if (len(msg) != parser.hdr_msgLen):
                rospy.loginfo("Packet Failed: Message length unexpected")
                continue

            # READ CRC
            chk = gpsSerial.read(4)
            if (not parser.VerifyChecksum(sync+header+msg,chk)):
                print("Packet Failed: CRC Did not Match")
                continue

            # PARSE MESSAGE
            timeNow = rospy.get_rostime()
            gpstime.header.stamp = timeNow
            if parser.hdr_msgID == parser.BESTPOS:
                #BESTPOS message
                navData.header.stamp = timeNow
                gpsStatus.header.stamp = timeNow
                parser.ParseBestPos(msg, navData, gpsStatus)
                parser.ParseTimeRef(gpstime)
            
                # Publish navData and time reference
                gpsPub.publish(navData)
                gpsStatPub.publish(gpsStatus)
                #gpsTimePub.publish(gpstime) #eh, dont publish time reference for now

            elif parser.hdr_msgID == 99:
                #BESTVEL message
                gpsVel.header.stamp = timeNow
                parser.ParseBestVel(msg,gpsVel)

                # Publish gpsVel
                gpsVelPub.publish(gpsVel)

            else:
                print("Novatel message ID not recognized. BESTPOS and BESTVEL supported only")
			"""
    except rospy.ROSInterruptException:
        gpsSerial.close() #Close gpsSerial serial port
