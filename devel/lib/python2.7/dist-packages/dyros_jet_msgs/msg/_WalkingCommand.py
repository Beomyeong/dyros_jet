# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from dyros_jet_msgs/WalkingCommand.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class WalkingCommand(genpy.Message):
  _md5sum = "7c98f46b805c25b4897e6489a2543122"
  _type = "dyros_jet_msgs/WalkingCommand"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """std_msgs/Header header

uint32 walk_mode
uint32 NONE = 0
uint32 STATIC_WALKING = 1

bool[2] compensator_mode
uint32 HIP_COMPENSTOR = 0
uint32 EXTERNAL_ENCODER = 1

uint32 ik_mode
uint32 IK=0
uint32 JACOBIAN = 1
uint32 QP = 2

bool heel_toe

bool first_foot_step

float32 x
float32 y
float32 z
float32 height
float32 theta
float32 step_length_x
float32 step_length_y

bool walking_pattern

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id
"""
  # Pseudo-constants
  NONE = 0
  STATIC_WALKING = 1
  HIP_COMPENSTOR = 0
  EXTERNAL_ENCODER = 1
  IK = 0
  JACOBIAN = 1
  QP = 2

  __slots__ = ['header','walk_mode','compensator_mode','ik_mode','heel_toe','first_foot_step','x','y','z','height','theta','step_length_x','step_length_y','walking_pattern']
  _slot_types = ['std_msgs/Header','uint32','bool[2]','uint32','bool','bool','float32','float32','float32','float32','float32','float32','float32','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,walk_mode,compensator_mode,ik_mode,heel_toe,first_foot_step,x,y,z,height,theta,step_length_x,step_length_y,walking_pattern

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(WalkingCommand, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.walk_mode is None:
        self.walk_mode = 0
      if self.compensator_mode is None:
        self.compensator_mode = [False] * 2
      if self.ik_mode is None:
        self.ik_mode = 0
      if self.heel_toe is None:
        self.heel_toe = False
      if self.first_foot_step is None:
        self.first_foot_step = False
      if self.x is None:
        self.x = 0.
      if self.y is None:
        self.y = 0.
      if self.z is None:
        self.z = 0.
      if self.height is None:
        self.height = 0.
      if self.theta is None:
        self.theta = 0.
      if self.step_length_x is None:
        self.step_length_x = 0.
      if self.step_length_y is None:
        self.step_length_y = 0.
      if self.walking_pattern is None:
        self.walking_pattern = False
    else:
      self.header = std_msgs.msg.Header()
      self.walk_mode = 0
      self.compensator_mode = [False] * 2
      self.ik_mode = 0
      self.heel_toe = False
      self.first_foot_step = False
      self.x = 0.
      self.y = 0.
      self.z = 0.
      self.height = 0.
      self.theta = 0.
      self.step_length_x = 0.
      self.step_length_y = 0.
      self.walking_pattern = False

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.walk_mode
      buff.write(_get_struct_I().pack(_x))
      buff.write(_get_struct_2B().pack(*self.compensator_mode))
      _x = self
      buff.write(_get_struct_I2B7fB().pack(_x.ik_mode, _x.heel_toe, _x.first_foot_step, _x.x, _x.y, _x.z, _x.height, _x.theta, _x.step_length_x, _x.step_length_y, _x.walking_pattern))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (self.walk_mode,) = _get_struct_I().unpack(str[start:end])
      start = end
      end += 2
      self.compensator_mode = _get_struct_2B().unpack(str[start:end])
      self.compensator_mode = list(map(bool, self.compensator_mode))
      _x = self
      start = end
      end += 35
      (_x.ik_mode, _x.heel_toe, _x.first_foot_step, _x.x, _x.y, _x.z, _x.height, _x.theta, _x.step_length_x, _x.step_length_y, _x.walking_pattern,) = _get_struct_I2B7fB().unpack(str[start:end])
      self.heel_toe = bool(self.heel_toe)
      self.first_foot_step = bool(self.first_foot_step)
      self.walking_pattern = bool(self.walking_pattern)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.walk_mode
      buff.write(_get_struct_I().pack(_x))
      buff.write(self.compensator_mode.tostring())
      _x = self
      buff.write(_get_struct_I2B7fB().pack(_x.ik_mode, _x.heel_toe, _x.first_foot_step, _x.x, _x.y, _x.z, _x.height, _x.theta, _x.step_length_x, _x.step_length_y, _x.walking_pattern))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (self.walk_mode,) = _get_struct_I().unpack(str[start:end])
      start = end
      end += 2
      self.compensator_mode = numpy.frombuffer(str[start:end], dtype=numpy.bool, count=2)
      self.compensator_mode = list(map(bool, self.compensator_mode))
      _x = self
      start = end
      end += 35
      (_x.ik_mode, _x.heel_toe, _x.first_foot_step, _x.x, _x.y, _x.z, _x.height, _x.theta, _x.step_length_x, _x.step_length_y, _x.walking_pattern,) = _get_struct_I2B7fB().unpack(str[start:end])
      self.heel_toe = bool(self.heel_toe)
      self.first_foot_step = bool(self.first_foot_step)
      self.walking_pattern = bool(self.walking_pattern)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2B = None
def _get_struct_2B():
    global _struct_2B
    if _struct_2B is None:
        _struct_2B = struct.Struct("<2B")
    return _struct_2B
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_I2B7fB = None
def _get_struct_I2B7fB():
    global _struct_I2B7fB
    if _struct_I2B7fB is None:
        _struct_I2B7fB = struct.Struct("<I2B7fB")
    return _struct_I2B7fB
