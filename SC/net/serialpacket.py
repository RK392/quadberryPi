PACKET_FIELD_SIZE = 4
PACKET_VALUE_SIZE = 6
PACKET_MAX_DATA_SIZE = 1024

FIELD_START = 'strt'
FIELD_TYPE = 'type' 
FIELD_SIZE = 'size'
FIELD_DATA = 'data'
FIELD_STOP = 'stop'

TYPE_CMD_UPDATE = 'updt'
TYPE_CMD_READ = 'read'

TYPE_ACK = 'ack'
TYPE_VALUE = 'val'

TYPE_ERR = 'err'


class PacketException(Exception):

    def __init__(self, message):
        super(PacketException, self).__init__(message)


# noinspection PyTypeChecker
class Packet:
        
    def __init__(self, type=None, data=None):
        self.packetName = 0
        self.packetNumber = 0
        self.type = type
        self.size = len(data) if data is not None else 0
        self.data = data

    def __repr__(self):
        return FIELD_START \
               + '|' + FIELD_TYPE + ':' + str(self.type) \
               + '|' + FIELD_SIZE + ':' + str(self.size) \
               + '|' + FIELD_DATA + ':' + str(self.data) \
               + '|' + FIELD_STOP

    def __str__(self):
        return 'Packet('+self.type+','+self.data+')'
        


def _read_string_field(conn):
    value = conn.recv(PACKET_VALUE_SIZE)
    return value.strip()
    
    
def _read_number_field(conn):
    value = conn.recv(PACKET_VALUE_SIZE)
    return int(value)    


def _write_value(conn, value):
    fvalue = value.rjust(6,' ') if isinstance(value, basestring) else str(value).rjust(6,'0')
    return conn.send(fvalue)


# Server side send, expects response
def send_command(conn, packet):
    write_packet(conn, packet)
    #sleep(0.005)
    return read_packet(conn)


# returns a packet
def read_packet(conn):
    found_packet = False
    packet = Packet()
    start1 = None
    while start1 != FIELD_START[0]:
        start1 = conn.recv(1)
        if start1 == FIELD_START[0]:
            start2 = conn.recv(PACKET_FIELD_SIZE-1)
            if start2 == FIELD_START[1:]:
                data_size = None
                field = None
                while field != FIELD_STOP:
                    field = conn.recv(PACKET_FIELD_SIZE)
                    if field == FIELD_TYPE:
                        type = _read_string_field(conn)
                        packet.type = type
                    elif field == FIELD_SIZE:
                        size = _read_number_field(conn)
                        data_size = size
                        packet.size = size
                    elif field == FIELD_DATA:
                        if data_size is not None:
                            data = conn.recv(data_size)
                            packet.data = data
                        else:
                            raise PacketException("Size not set in packet!")
                    elif field == FIELD_STOP:
                        found_packet = True
                        pass
                    else:
                        raise PacketException("Invalid field: " + field)
    if not found_packet:
        raise PacketException("Packet not found!")
    return packet


def write_packet(conn,packet):
    conn.send(FIELD_START)
    conn.send(FIELD_TYPE)
    _write_value(conn, packet.type)
    conn.send(FIELD_SIZE)
    _write_value(conn, packet.size)
    conn.send(FIELD_DATA)
    conn.send(packet.data)
    conn.send(FIELD_STOP)