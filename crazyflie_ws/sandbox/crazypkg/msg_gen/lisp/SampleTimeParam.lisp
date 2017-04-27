; Auto-generated. Do not edit!


(cl:in-package crazypkg-msg)


;//! \htmlinclude SampleTimeParam.msg.html

(cl:defclass <SampleTimeParam> (roslisp-msg-protocol:ros-message)
  ((sampleTimeType
    :reader sampleTimeType
    :initarg :sampleTimeType
    :type cl:fixnum
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0))
)

(cl:defclass SampleTimeParam (<SampleTimeParam>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SampleTimeParam>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SampleTimeParam)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crazypkg-msg:<SampleTimeParam> is deprecated: use crazypkg-msg:SampleTimeParam instead.")))

(cl:ensure-generic-function 'sampleTimeType-val :lambda-list '(m))
(cl:defmethod sampleTimeType-val ((m <SampleTimeParam>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazypkg-msg:sampleTimeType-val is deprecated.  Use crazypkg-msg:sampleTimeType instead.")
  (sampleTimeType m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <SampleTimeParam>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazypkg-msg:value-val is deprecated.  Use crazypkg-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SampleTimeParam>) ostream)
  "Serializes a message object of type '<SampleTimeParam>"
  (cl:let* ((signed (cl:slot-value msg 'sampleTimeType)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SampleTimeParam>) istream)
  "Deserializes a message object of type '<SampleTimeParam>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sampleTimeType) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SampleTimeParam>)))
  "Returns string type for a message object of type '<SampleTimeParam>"
  "crazypkg/SampleTimeParam")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SampleTimeParam)))
  "Returns string type for a message object of type 'SampleTimeParam"
  "crazypkg/SampleTimeParam")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SampleTimeParam>)))
  "Returns md5sum for a message object of type '<SampleTimeParam>"
  "976f518ece3cd97bf9e643ee0b1ebe6d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SampleTimeParam)))
  "Returns md5sum for a message object of type 'SampleTimeParam"
  "976f518ece3cd97bf9e643ee0b1ebe6d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SampleTimeParam>)))
  "Returns full string definition for message of type '<SampleTimeParam>"
  (cl:format cl:nil "int16 sampleTimeType~%float64 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SampleTimeParam)))
  "Returns full string definition for message of type 'SampleTimeParam"
  (cl:format cl:nil "int16 sampleTimeType~%float64 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SampleTimeParam>))
  (cl:+ 0
     2
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SampleTimeParam>))
  "Converts a ROS message object to a list"
  (cl:list 'SampleTimeParam
    (cl:cons ':sampleTimeType (sampleTimeType msg))
    (cl:cons ':value (value msg))
))
