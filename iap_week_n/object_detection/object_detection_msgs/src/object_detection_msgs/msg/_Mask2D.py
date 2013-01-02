"""autogenerated by genmsg_py from Mask2D.msg. Do not edit."""
import roslib.message
import struct


class Mask2D(roslib.message.Message):
  _md5sum = "c073105a22d894f799b67a16c88b190a"
  _type = "object_detection_msgs/Mask2D"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """uint32 height
uint32 width
bool[] isValid

"""
  __slots__ = ['height','width','isValid']
  _slot_types = ['uint32','uint32','bool[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       height,width,isValid
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(Mask2D, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.height is None:
        self.height = 0
      if self.width is None:
        self.width = 0
      if self.isValid is None:
        self.isValid = []
    else:
      self.height = 0
      self.width = 0
      self.isValid = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_2I.pack(_x.height, _x.width))
      length = len(self.isValid)
      buff.write(_struct_I.pack(length))
      pattern = '<%sB'%length
      buff.write(struct.pack(pattern, *self.isValid))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      _x = self
      start = end
      end += 8
      (_x.height, _x.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sB'%length
      start = end
      end += struct.calcsize(pattern)
      self.isValid = struct.unpack(pattern, str[start:end])
      self.isValid = map(bool, self.isValid)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_2I.pack(_x.height, _x.width))
      length = len(self.isValid)
      buff.write(_struct_I.pack(length))
      pattern = '<%sB'%length
      buff.write(self.isValid.tostring())
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 8
      (_x.height, _x.width,) = _struct_2I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sB'%length
      start = end
      end += struct.calcsize(pattern)
      self.isValid = numpy.frombuffer(str[start:end], dtype=numpy.bool, count=length)
      self.isValid = map(bool, self.isValid)
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_2I = struct.Struct("<2I")
