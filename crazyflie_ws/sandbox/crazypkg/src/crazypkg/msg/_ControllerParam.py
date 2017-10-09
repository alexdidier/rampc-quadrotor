# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from crazypkg/ControllerParam.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class ControllerParam(genpy.Message):
  _md5sum = "4b31db51200388b6429184e9d083fa8c"
  _type = "crazypkg/ControllerParam"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int16 crazyControllerType
int16 basicControllerType
int16 paramType
float64 value
"""
  __slots__ = ['crazyControllerType','basicControllerType','paramType','value']
  _slot_types = ['int16','int16','int16','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       crazyControllerType,basicControllerType,paramType,value

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ControllerParam, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.crazyControllerType is None:
        self.crazyControllerType = 0
      if self.basicControllerType is None:
        self.basicControllerType = 0
      if self.paramType is None:
        self.paramType = 0
      if self.value is None:
        self.value = 0.
    else:
      self.crazyControllerType = 0
      self.basicControllerType = 0
      self.paramType = 0
      self.value = 0.

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
      buff.write(_get_struct_3hd().pack(_x.crazyControllerType, _x.basicControllerType, _x.paramType, _x.value))
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
      end += 14
      (_x.crazyControllerType, _x.basicControllerType, _x.paramType, _x.value,) = _get_struct_3hd().unpack(str[start:end])
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
      buff.write(_get_struct_3hd().pack(_x.crazyControllerType, _x.basicControllerType, _x.paramType, _x.value))
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
      end += 14
      (_x.crazyControllerType, _x.basicControllerType, _x.paramType, _x.value,) = _get_struct_3hd().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3hd = None
def _get_struct_3hd():
    global _struct_3hd
    if _struct_3hd is None:
        _struct_3hd = struct.Struct("<3hd")
    return _struct_3hd
