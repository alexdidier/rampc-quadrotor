# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from crazypkg/MotorCommands.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class MotorCommands(genpy.Message):
  _md5sum = "1df7dce4d8fafaa23f1189b3eaa4180b"
  _type = "crazypkg/MotorCommands"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float32 cmd1
float32 cmd2
float32 cmd3
float32 cmd4

"""
  __slots__ = ['cmd1','cmd2','cmd3','cmd4']
  _slot_types = ['float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       cmd1,cmd2,cmd3,cmd4

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MotorCommands, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.cmd1 is None:
        self.cmd1 = 0.
      if self.cmd2 is None:
        self.cmd2 = 0.
      if self.cmd3 is None:
        self.cmd3 = 0.
      if self.cmd4 is None:
        self.cmd4 = 0.
    else:
      self.cmd1 = 0.
      self.cmd2 = 0.
      self.cmd3 = 0.
      self.cmd4 = 0.

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
      buff.write(_get_struct_4f().pack(_x.cmd1, _x.cmd2, _x.cmd3, _x.cmd4))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 16
      (_x.cmd1, _x.cmd2, _x.cmd3, _x.cmd4,) = _get_struct_4f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_4f().pack(_x.cmd1, _x.cmd2, _x.cmd3, _x.cmd4))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 16
      (_x.cmd1, _x.cmd2, _x.cmd3, _x.cmd4,) = _get_struct_4f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_4f = None
def _get_struct_4f():
    global _struct_4f
    if _struct_4f is None:
        _struct_4f = struct.Struct("<4f")
    return _struct_4f
