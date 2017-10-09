; Auto-generated. Do not edit!


(cl:in-package crazypkg-msg)


;//! \htmlinclude ControllerOutputPackage.msg.html

(cl:defclass <ControllerOutputPackage> (roslisp-msg-protocol:ros-message)
  ((roll
    :reader roll
    :initarg :roll
    :type cl:float
    :initform 0.0)
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (thrust
    :reader thrust
    :initarg :thrust
    :type cl:fixnum
    :initform 0)
   (motorCmd1
    :reader motorCmd1
    :initarg :motorCmd1
    :type cl:fixnum
    :initform 0)
   (motorCmd2
    :reader motorCmd2
    :initarg :motorCmd2
    :type cl:fixnum
    :initform 0)
   (motorCmd3
    :reader motorCmd3
    :initarg :motorCmd3
    :type cl:fixnum
    :initform 0)
   (motorCmd4
    :reader motorCmd4
    :initarg :motorCmd4
    :type cl:fixnum
    :initform 0)
   (onboardControllerType
    :reader onboardControllerType
    :initarg :onboardControllerType
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ControllerOutputPackage (<ControllerOutputPackage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControllerOutputPackage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControllerOutputPackage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crazypkg-msg:<ControllerOutputPackage> is deprecated: use crazypkg-msg:ControllerOutputPackage instead.")))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <ControllerOutputPackage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazypkg-msg:roll-val is deprecated.  Use crazypkg-msg:roll instead.")
  (roll m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <ControllerOutputPackage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazypkg-msg:pitch-val is deprecated.  Use crazypkg-msg:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <ControllerOutputPackage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazypkg-msg:yaw-val is deprecated.  Use crazypkg-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'thrust-val :lambda-list '(m))
(cl:defmethod thrust-val ((m <ControllerOutputPackage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazypkg-msg:thrust-val is deprecated.  Use crazypkg-msg:thrust instead.")
  (thrust m))

(cl:ensure-generic-function 'motorCmd1-val :lambda-list '(m))
(cl:defmethod motorCmd1-val ((m <ControllerOutputPackage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazypkg-msg:motorCmd1-val is deprecated.  Use crazypkg-msg:motorCmd1 instead.")
  (motorCmd1 m))

(cl:ensure-generic-function 'motorCmd2-val :lambda-list '(m))
(cl:defmethod motorCmd2-val ((m <ControllerOutputPackage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazypkg-msg:motorCmd2-val is deprecated.  Use crazypkg-msg:motorCmd2 instead.")
  (motorCmd2 m))

(cl:ensure-generic-function 'motorCmd3-val :lambda-list '(m))
(cl:defmethod motorCmd3-val ((m <ControllerOutputPackage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazypkg-msg:motorCmd3-val is deprecated.  Use crazypkg-msg:motorCmd3 instead.")
  (motorCmd3 m))

(cl:ensure-generic-function 'motorCmd4-val :lambda-list '(m))
(cl:defmethod motorCmd4-val ((m <ControllerOutputPackage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazypkg-msg:motorCmd4-val is deprecated.  Use crazypkg-msg:motorCmd4 instead.")
  (motorCmd4 m))

(cl:ensure-generic-function 'onboardControllerType-val :lambda-list '(m))
(cl:defmethod onboardControllerType-val ((m <ControllerOutputPackage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazypkg-msg:onboardControllerType-val is deprecated.  Use crazypkg-msg:onboardControllerType instead.")
  (onboardControllerType m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControllerOutputPackage>) ostream)
  "Serializes a message object of type '<ControllerOutputPackage>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'thrust)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'thrust)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'motorCmd1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'motorCmd1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'motorCmd2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'motorCmd2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'motorCmd3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'motorCmd3)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'motorCmd4)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'motorCmd4)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'onboardControllerType)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControllerOutputPackage>) istream)
  "Deserializes a message object of type '<ControllerOutputPackage>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'thrust)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'thrust)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'motorCmd1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'motorCmd1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'motorCmd2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'motorCmd2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'motorCmd3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'motorCmd3)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'motorCmd4)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'motorCmd4)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'onboardControllerType)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControllerOutputPackage>)))
  "Returns string type for a message object of type '<ControllerOutputPackage>"
  "crazypkg/ControllerOutputPackage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControllerOutputPackage)))
  "Returns string type for a message object of type 'ControllerOutputPackage"
  "crazypkg/ControllerOutputPackage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControllerOutputPackage>)))
  "Returns md5sum for a message object of type '<ControllerOutputPackage>"
  "0b6ac24b895aa888e2867a1a58e54d9e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControllerOutputPackage)))
  "Returns md5sum for a message object of type 'ControllerOutputPackage"
  "0b6ac24b895aa888e2867a1a58e54d9e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControllerOutputPackage>)))
  "Returns full string definition for message of type '<ControllerOutputPackage>"
  (cl:format cl:nil "float32 roll~%float32 pitch~%float32 yaw~%uint16 thrust~%uint16 motorCmd1~%uint16 motorCmd2~%uint16 motorCmd3~%uint16 motorCmd4~%uint8 onboardControllerType~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControllerOutputPackage)))
  "Returns full string definition for message of type 'ControllerOutputPackage"
  (cl:format cl:nil "float32 roll~%float32 pitch~%float32 yaw~%uint16 thrust~%uint16 motorCmd1~%uint16 motorCmd2~%uint16 motorCmd3~%uint16 motorCmd4~%uint8 onboardControllerType~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControllerOutputPackage>))
  (cl:+ 0
     4
     4
     4
     2
     2
     2
     2
     2
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControllerOutputPackage>))
  "Converts a ROS message object to a list"
  (cl:list 'ControllerOutputPackage
    (cl:cons ':roll (roll msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':thrust (thrust msg))
    (cl:cons ':motorCmd1 (motorCmd1 msg))
    (cl:cons ':motorCmd2 (motorCmd2 msg))
    (cl:cons ':motorCmd3 (motorCmd3 msg))
    (cl:cons ':motorCmd4 (motorCmd4 msg))
    (cl:cons ':onboardControllerType (onboardControllerType msg))
))
