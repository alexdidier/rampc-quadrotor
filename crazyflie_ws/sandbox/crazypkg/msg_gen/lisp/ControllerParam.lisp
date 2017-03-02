; Auto-generated. Do not edit!


(cl:in-package crazypkg-msg)


;//! \htmlinclude ControllerParam.msg.html

(cl:defclass <ControllerParam> (roslisp-msg-protocol:ros-message)
  ((crazyControllerType
    :reader crazyControllerType
    :initarg :crazyControllerType
    :type cl:fixnum
    :initform 0)
   (basicControllerType
    :reader basicControllerType
    :initarg :basicControllerType
    :type cl:fixnum
    :initform 0)
   (paramType
    :reader paramType
    :initarg :paramType
    :type cl:fixnum
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0))
)

(cl:defclass ControllerParam (<ControllerParam>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerParam>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerParam)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crazypkg-msg:<ControllerParam> is deprecated: use crazypkg-msg:ControllerParam instead.")))

(cl:ensure-generic-function 'crazyControllerType-val :lambda-list '(m))
(cl:defmethod crazyControllerType-val ((m <ControllerParam>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazypkg-msg:crazyControllerType-val is deprecated.  Use crazypkg-msg:crazyControllerType instead.")
  (crazyControllerType m))

(cl:ensure-generic-function 'basicControllerType-val :lambda-list '(m))
(cl:defmethod basicControllerType-val ((m <ControllerParam>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazypkg-msg:basicControllerType-val is deprecated.  Use crazypkg-msg:basicControllerType instead.")
  (basicControllerType m))

(cl:ensure-generic-function 'paramType-val :lambda-list '(m))
(cl:defmethod paramType-val ((m <ControllerParam>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazypkg-msg:paramType-val is deprecated.  Use crazypkg-msg:paramType instead.")
  (paramType m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <ControllerParam>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazypkg-msg:value-val is deprecated.  Use crazypkg-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerParam>) ostream)
  "Serializes a message object of type '<ControllerParam>"
  (cl:let* ((signed (cl:slot-value msg 'crazyControllerType)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'basicControllerType)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'paramType)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerParam>) istream)
  "Deserializes a message object of type '<ControllerParam>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'crazyControllerType) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'basicControllerType) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'paramType) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerParam>)))
  "Returns string type for a message object of type '<ControllerParam>"
  "crazypkg/ControllerParam")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerParam)))
  "Returns string type for a message object of type 'ControllerParam"
  "crazypkg/ControllerParam")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerParam>)))
  "Returns md5sum for a message object of type '<ControllerParam>"
  "4b31db51200388b6429184e9d083fa8c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerParam)))
  "Returns md5sum for a message object of type 'ControllerParam"
  "4b31db51200388b6429184e9d083fa8c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerParam>)))
  "Returns full string definition for message of type '<ControllerParam>"
  (cl:format cl:nil "int16 crazyControllerType~%int16 basicControllerType~%int16 paramType~%float64 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerParam)))
  "Returns full string definition for message of type 'ControllerParam"
  (cl:format cl:nil "int16 crazyControllerType~%int16 basicControllerType~%int16 paramType~%float64 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerParam>))
  (cl:+ 0
     2
     2
     2
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerParam>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerParam
    (cl:cons ':crazyControllerType (crazyControllerType msg))
    (cl:cons ':basicControllerType (basicControllerType msg))
    (cl:cons ':paramType (paramType msg))
    (cl:cons ':value (value msg))
))
